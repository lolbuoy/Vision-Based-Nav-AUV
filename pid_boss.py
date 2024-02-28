import math
# Import mavutil
from pymavlink import mavutil
# Imports for attitude
from pymavlink.quaternion import QuaternionBase
import os
import csv
import time
import csvreader
import random
class PIDController:
    def __init__(self, Kp_roll, Ki_roll, Kd_roll,Kp_pitch,Ki_pitch,Kd_pitch,setpoint, sample_time):
        # PID parameters for roll axis
        self.Kp_roll = Kp_roll
        self.Ki_roll = Ki_roll
        self.Kd_roll = Kd_roll
        self.Kp_pitch = Kp_pitch
        self.Ki_pitch = Ki_pitch
        self.Kd_pitch = Kd_pitch

        self.setpoint = setpoint
        self.sample_time = sample_time
        self.prev_time = time.time()

        # Initialize previous error and integral terms for roll
        self.prev_error_roll = 0
        self.integral_roll = 0
        self.count = 0
        self.prev_error_pitch = 0
        self.integral_pitch = 0
        # Initialize output for roll
        self.output_roll = 1500  # Initial output set to 1500
        self.output_pitch = 1500
        # Create indexed CSV file for logging
        self.roll_log_index = 0

    def update(self, rollinput):
        self.count +=1
        # Compute time difference since last update
        current_time = time.time()
        dt = current_time - self.prev_time
        print(current_time,self.prev_time)

        # Check if enough time has passed for a new update
        if dt >= self.sample_time:
            # Compute error terms for roll axis
            error_roll = self.setpoint - rollinput[0]
            self.integral_roll += error_roll * dt
            derivative_roll = (error_roll - self.prev_error_roll) / dt
            error_pitch = self.setpoint - rollinput[2]
            self.integral_pitch += error_roll * dt
            derivative_pitch = (error_roll - self.prev_error_pitch) / dt

            # Compute PID output for roll axis
            self.output_roll = (
                self.Kp_roll * error_roll +
                self.Ki_roll * self.integral_roll +
                self.Kd_roll * derivative_roll
            )
            self.output_pitch = (
                self.Kp_pitch * error_pitch +
                self.Ki_pitch * self.integral_pitch +
                self.Kd_pitch * derivative_pitch
            )
            self.output_pitch = int(max(1000, min(2000, 1500-self.output_pitch)))
            # Adjust output based on rollinput
            if rollinput[0] <= 0.5:
                print("case 1 ", rollinput, self.output_roll)
                self.output_roll = int(max(1490, min(1510, 1510+self.output_roll)))
            elif rollinput[0] == 0:
                print("case 2 ", rollinput, self.output_roll)
                self.output_roll = 1500
            else:
                if rollinput[1] == 1:
                    print("case 3 ", rollinput, self.output_roll)
                    self.output_roll = int(max(1510, min(2000, 2000 + self.output_roll)))
                else:
                    print("case 4 ", rollinput, self.output_roll)
                    self.output_roll = int(max(1000, min(1490, 1490 + self.output_roll)))

            # Log the data for roll
            print(self.count)
            self.log_data('roll', current_time, [self.output_roll,self.output_pitch] , rollinput)
            time.sleep(0.095)
            # Update previous time and error for next iteration
            self.prev_time = current_time
            self.prev_error_roll = error_roll
            self.prev_error_pitch = error_pitch

        return [self.output_roll,self.output_pitch]

    def log_data(self, axis, timestamp, output, input):
        # Create indexed CSV file for logging
        if axis == 'roll':
            log_file = f'roll_log_th8.csv'

        with open(log_file, 'a', newline='') as file:
            log_writer = csv.writer(file)
            log_writer.writerow([timestamp, output[0], input[0],input[1],output[1],input[2]])

def set_rc_channel_pwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)

def set_target_depth(depth):
    """ Sets the target depth while in depth-hold mode.

    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

    'depth' is technically an altitude, so set as negative meters below the surface
        -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

    """
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=( # ignore everything except z position
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ), lat_int=0, lon_int=0, alt=depth, # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )

def set_target_attitude(roll, pitch, yaw):
    """ Sets the target attitude while in depth-hold mode.

    'roll', 'pitch', and 'yaw' are angles in degrees.

    """
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
    )

master = mavutil.mavlink_connection('tcp:0.0.0.0:5763')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()
# arm ArduSub autopilot and wait until confirmed
master.arducopter_arm()
master.motors_armed_wait()

# set the desired operating mode
DEPTH_HOLD = 'ALT_HOLD'
DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    master.set_mode(DEPTH_HOLD)

# set a depth target
set_target_depth(-1.75)

# Read data from CSV file
data = []
leftright  = csvreader.process_image_distances('generated_data3.csv')
# Iterate ov#er the input dictionary
#print(leftright)
prev = 0
for item in leftright:
    left = item['left_distance']
    right = item['right_distance']
    front = item['front_distance']
    rollinput = 0.5 * abs((left * left) - (right * right)) ** 0.5
    pitchinput = front
    if left >=right:
        data.append([rollinput,0,pitchinput,0])
    else:
        data.append([rollinput,1,pitchinput,0])


# Define PID parameters for roll axis
Kp_roll = 0.5
Ki_roll = 0.5
Kd_roll = 0.01
Kp_pitch = 0.5
Ki_pitch = .5
Kd_pitch = 0.01
# Define setpoint and sample time
setpoint = 0.1  # Adjust as per your requirement
sample_time = 0.01  # 10 ms
time.sleep(1)

# Initialize PID controller
pid = PIDController(
    Kp_roll, Ki_roll, Kd_roll,
    Kp_pitch, Ki_pitch, Kd_pitch,
    setpoint, sample_time
)

# Process data and apply PID controller
for rollinput in data:
    # Compute PID output based on current measurement
    timerandom = random.uniform(0.007,0.012)
    print(rollinput)
    time.sleep(timerandom)
    pid_output = pid.update(rollinput)

    # Apply PID output to your system (e.g., set PWM value)
    # For this example, we're just printing out the PID output
    print("Roll PID Output:", pid_output[0])
    set_rc_channel_pwm(2,pid_output[0])
    print("Pitch PID Output:", pid_output[1])
    set_rc_channel_pwm(1,pid_output[1])