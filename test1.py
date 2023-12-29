import RPi.GPIO as GPIO
import time
import keyboard
#import automove
from pymavlink import mavutil
# Imports for attitude
from pymavlink.quaternion import QuaternionBase

test_depth=-2.5
# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pin for the switch
switch_pin = 17

# Set up the switch as input with a pull-up resistor
GPIO.setup(switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Variables to track switch states
switch_state = "CENTER"

# Define functions for each switch state
def set_desired_movement(x=0, y=0, z=0, yaw=0, button=0):
    master.mav.manual_control_send(master.target_system, x, y, z, yaw, button)

def set_target_depth(depth):
    master.mav.set_position_target_local_ned_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=1,
        type_mask=(  # ignore everything except z position
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ),
        x=0.0, y=0.0, z=depth,
        vx=0, vy=0, vz=0,
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
    )

def do_automation(test_depth):
    set_target_depth(test_depth)
    i = 0

    if switch_state != "OFF" and i == 0:
        # Arm the vehicle and wait for motors to be armed
        master.arducopter_arm()
        master.motors_armed_wait()
        print("Armed")
        print("Moving ahead for 5 Seconds")
        # Move forward for 5 seconds while maintaining depth
        start_time = time.time()
        while time.time() - start_time < 5 and switch_state != "OFF":
            set_desired_movement(x=500)
            set_target_depth(test_depth)
            time.sleep(0.1)  # Adjust sleep duration as needed

        # Yaw 180 degrees while maintaining depth
        print("Yawing 180")
        yaw_start_time = time.time()
        while time.time() - yaw_start_time < 2 and switch_state != "OFF":
            set_rc_channel_pwm(4, 2000)  # Adjust as needed for your setup
            set_target_depth(test_depth)
            time.sleep(0.1)  # Adjust sleep duration as needed
            
        print("Moving ahead for 5 Seconds")
        # Move forward for another 5 seconds while maintaining depth
        start_time = time.time()
        while time.time() - start_time < 5 and switch_state != "OFF":
            set_desired_movement(x=300)
            set_target_depth(test_depth)
            time.sleep(0.1)  # Adjust sleep duration as needed

        # Stop the movement after 5 seconds
        set_desired_movement(x=0)
        print("Stopped movement")
        i += 1

    # Additional loop to maintain depth for 4 seconds after the main actions
    for _ in range(40):
        set_target_depth(test_depth)
        time.sleep(0.1)
    i += 1




def switch_turned_on():
    print("Switch turned ON!")
    do_automation()

def switch_turned_off():
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Switch turned OFF!")
    print("DISARMED")

def switch_in_center():
    print("Switch in the center position.")

master = mavutil.mavlink_connection('localhost:6900')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()
print("heartbeat")

# arm ArduSub autopilot and wait until confirmed

def set_rc_channel_pwm(channel_id, pwm=1500):
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


# set the desired operating mode
DEPTH_HOLD = 'ALT_HOLD'
DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    master.set_mode(DEPTH_HOLD)
try:
    while True:
        current_switch_state = GPIO.input(switch_pin)

        if current_switch_state == GPIO.LOW and switch_state != "ON":
            switch_state = "ON"
            do_automation(test_depth)
            

        elif current_switch_state == GPIO.HIGH and switch_state != "OFF":
            switch_turned_off()
            switch_state = "OFF"

        time.sleep(0.1)

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
