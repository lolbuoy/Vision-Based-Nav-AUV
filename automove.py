
import time
import keyboard
import math
# Import mavutil
from pymavlink import mavutil
# Imports for attitude
from pymavlink.quaternion import QuaternionBase

test_depth=1.5

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
        vx=0, vy=0, vz=0.5, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )
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
        *rc_channel_values)                  # RC channel list, in microseconds.



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

# Create the connection
master = mavutil.mavlink_connection('tcp:0.0.0.0:5763')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()
print("heartbeat")

# arm ArduSub autopilot and wait until confirmed
master.arducopter_arm()
master.motors_armed_wait()
print("armed")
# set the desired operating mode
DEPTH_HOLD = 'ALT_HOLD'
DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    master.set_mode(DEPTH_HOLD)
def set_desired_movement(x=0,y=0,z=0,yaw=0,button=0):
    master.mav.manual_control_send(
    master.target_system,
    x,
    y,
    z,
    yaw,
    button)

# set a depth target    
def do_automation(test_depth):
    # Set the depth target outside the loop to make it constant
    set_target_depth(test_depth)

    time.sleep(10)
    print("going ahead for 5 seconds")
    set_desired_movement(500)
    time.sleep(5)
    set_rc_channel_pwm(4, 2000)
    time.sleep(2)
    set_desired_movement(500)
    time.sleep(5)

    # Reset the depth target to the original value after the automation routine
    set_target_depth(test_depth)



# go for a spin
# (set target yaw from 0 to 500 degrees in steps of 10, one update per second)
#roll_angle = pitch_angle = 0
#for yaw_angle in range(0, 500, 10):
#    set_target_attitude(roll_angle, pitch_angle, yaw_angle)
#    time.sleep(1) # wait for a second

# spin the other way with 3x larger steps
#for yaw_angle in range(500, 0, -30):
#    set_target_attitude(roll_angle, pitch_angle, yaw_angle)
#    time.sleep(1)

# clean up (disarm) at the end
#master.arducopter_disarm()
#master.motors_disarmed_wait()
# Function to send manual control inputs based on keyboard events
def send_manual_control_inputs():
    current_depth = 0.0  # Initialize depth to 0.0 meters
    depth_change = 0.5  # Depth change per key press

    while True:
        try:
            if keyboard.is_pressed('w'):
                master.mav.manual_control_send(master.target_system, 0, -500, 500, 0, 0)
            elif keyboard.is_pressed('a'):
                master.mav.manual_control_send(master.target_system, -500, 0, 500, 0, 0)
            elif keyboard.is_pressed('s'):
                master.mav.manual_control_send(master.target_system, 0, 500, 500, 0, 0)
            elif keyboard.is_pressed('d'):
                master.mav.manual_control_send(master.target_system, 500, 0, 500, 0, 0)
            elif keyboard.is_pressed('k'):
                master.mav.manual_control_send(master.target_system, 0, 0, 0, 0, 0)
                set_target_depth(1000)
                time.sleep(20)
                master.arducopter_disarm()
                master.motors_disarmed_wait()
            elif keyboard.is_pressed('q'):
                current_depth -= depth_change
                set_target_depth(current_depth)
                print(f"Depth increased to {current_depth} meters")
                time.sleep(0.5)  # Adjust sleep time as needed
            elif keyboard.is_pressed('z'):
                current_depth += depth_change
                set_target_depth(current_depth)
                print(f"Depth creased to {current_depth} meters")
                time.sleep(0.5)  # Adjust sleep time as needed
           
            elif keyboard.is_pressed('m'):
                 do_automation(test_depth)	

        except:
            break


# Start a thread to continuously check for keyboard events
keyboard_thread = keyboard.start_recording()

# Start sending manual control inputs
send_manual_control_inputs()



# Allow some time for manual control and depth adjustment
time.sleep(10000)

# Stop the keyboard thread
keyboard.stop_recording(keyboard_thread)

# Close the connection
master.close()
