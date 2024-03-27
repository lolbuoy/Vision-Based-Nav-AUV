import math
import rospy
from std_msgs.msg import Float32, UInt16
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import os
import csv
import time
import random

# Global variables for roll and distance values
roll_value = 0
distance_value = 0.0

# Callback function to update roll value
def roll_callback(msg):
    global roll_value
    roll_value = msg.data
    print("Roll value updated:", roll_value)

# Callback function to update distance value
def distance_callback(msg):
    global distance_value
    distance_value = msg.data
    print("Distance value updated:", distance_value)

class PIDController:
    def __init__(self, Kp_roll, Ki_roll, Kd_roll, Kp_pitch, Ki_pitch, Kd_pitch, setpoint_roll, setpoint_pitch, sample_time, log_folder):
        # PID parameters for roll axis
        self.Kp_roll = Kp_roll
        self.Ki_roll = Ki_roll
        self.Kd_roll = Kd_roll
        self.Kp_pitch = Kp_pitch
        self.Ki_pitch = Ki_pitch
        self.Kd_pitch = Kd_pitch

        self.setpoint_roll = setpoint_roll
        self.setpoint_pitch = setpoint_pitch
        self.sample_time = sample_time
        self.prev_time = time.time()
        self.log_folder = log_folder  # Log folder path

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
        self.count += 1
        # Compute time difference since last update
        current_time = time.time()
        dt = current_time - self.prev_time
        print("Current time:", current_time, "Previous time:", self.prev_time)

        # Check if enough time has passed for a new update
        if dt >= self.sample_time:
            # Compute error terms for roll axis
            error_roll = self.setpoint_roll - rollinput[0]
            self.integral_roll += error_roll * dt
            derivative_roll = (error_roll - self.prev_error_roll) / dt
            error_pitch = self.setpoint_pitch - rollinput[1]
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
            
            self.output_pitch = int(max(1000, min(2000, 1500 - self.output_pitch)))
            # Check if roll input is within the specified range
            if 200 <= rollinput[0] <= 440:
                if rollinput[0] <= 320:
                    # Do a certain action if roll input is less than or equal to 320
                    self.output_roll = int(max(1500, min(2000, 2000- self.output_pitch)))
                    # Perform action for this case
                else:
                    # Do a different action if roll input is greater than 320
                    self.output_roll = int(max(1000, min(1500, 1500 - self.output_pitch)))
                    # Perform action for this case
            else:
                # If roll input is not within the specified range, pass
                self.output_pitch = int(max(1000, min(2000, 1500 - self.output_pitch)))
                pass

            # Log the data for roll
            print("Count:", self.count)
            self.log_data('roll', current_time, [self.output_roll, self.output_pitch], rollinput)
            time.sleep(0.095)
            # Update previous time and error for next iteration
            self.prev_time = current_time
            self.prev_error_roll = error_roll
            self.prev_error_pitch = error_pitch

        return [self.output_roll, self.output_pitch]

    def log_data(self, axis, timestamp, output, input):
        # Create folder if it doesn't exist
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)

        # Generate log file name in the format ddmmyy_hr:min:sec.csv
        log_file = time.strftime("%d%m%y_%H:%M:%S") + '.csv'
        log_file_path = os.path.join(self.log_folder, log_file)

        with open(log_file_path, 'a', newline='') as file:
            log_writer = csv.writer(file)
            log_writer.writerow([timestamp, output[0], input[0], output[1], input[1]])

def main():
    # Initialize ROS node
    rospy.init_node('pid_controller_node', anonymous=True)

    # Define PID parameters for roll axis
    Kp_roll = 0.5
    Ki_roll = 0.5
    Kd_roll = 0.01
    Kp_pitch = 0.5
    Ki_pitch = 0.5
    Kd_pitch = 0.01
    # Define setpoint and sample time
    setpoint_roll = 320
    setpoint_pitch = 0.8
    # Adjust as per your requirement
    sample_time = 0.01  # 10 ms
    log_folder = '/home/vyana/logs/dive_flare_logs/'  # Log folder path

    # Initialize PID controller
    pid = PIDController(
        Kp_roll, Ki_roll, Kd_roll,
        Kp_pitch, Ki_pitch, Kd_pitch,
        setpoint_roll, setpoint_pitch, sample_time,
        log_folder=log_folder
    )

    # Subscribe to roll and distance topics
    rospy.Subscriber('/custom_node/flare_pixel_x', UInt16, roll_callback)
    rospy.Subscriber('/custom_node/flare_distance', Float32, distance_callback)

    # Process data and apply PID controller
    while not rospy.is_shutdown():
        # Compute PID output based on current measurement
        timerandom = random.uniform(0.1, 0.5)
        print("Roll value:", roll_value, "Distance value:", distance_value)
        time.sleep(timerandom)
        pid_output = pid.update([roll_value, distance_value])

        # Apply PID output to your system (e.g., set PWM value)
        # For this example, we're just printing out the PID output
        print("Roll PID Output:", pid_output[0])
        # set_rc_channel_pwm(2,pid_output[0])
        print("Pitch PID Output:", pid_output[1])
        # set_rc_channel_pwm(1,pid_output[1])

if __name__ == '__main__':
    print("PID CONTROLLER")
    main()
