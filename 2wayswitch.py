import RPi.GPIO as GPIO
import time
from pymavlink import mavutil

# GPIO pin and timing constants
BUTTON_PIN = 27                # Declare GPIO 27 for the button input
BUTTON_PRESS_TIME = 3          # Button press state time in seconds

# Set up GPIO configuration
GPIO.setmode(GPIO.BCM)          # Use BCM GPIO numbering
GPIO.setwarnings(False)         # Disable GPIO warnings
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Set up the button pin
time.sleep(1)  # Allow system initialization time

# Set up MAVLink connection
master = mavutil.mavlink_connection('udpin:192.168.1.39:4200')
boot_time = time.time()

master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))

# Set the mode string for depth hold
DEPTH_HOLD_MODE = 'ALT_HOLD'

# Set up the button as input with a pull-up resistor
button_pin = 17
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Variables to track button states
button_state = "CENTER"
armed = False  # Initialize the armed state

# Variable for test depth
test_depth = -2.5

# Function to set RC channel PWM
def set_rc_channel_pwm(channel_id, pwm=1500):
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_channel_values
    )

# Function to set desired movement
def set_desired_movement(x=0, y=0, z=0, yaw=0, button=0):
    master.mav.manual_control_send(master.target_system, x, y, z, yaw, button)

# Function to set target depth
def set_target_depth(depth):
    master.mav.set_position_target_local_ned_send(
        int(1e3 * (time.time() - boot_time)),
        master.target_system, master.target_component,
        coordinate_frame=1,
        type_mask=(
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

# Function to perform automation
def do_automation(test_depth):
    set_target_depth(test_depth)
    i = 0

    master.arducopter_arm()
    master.motors_armed_wait()
    print("Armed")

    start_time = time.time()
    while time.time() - start_time < 5 and button_state != "OFF":
        set_desired_movement(x=500)
        set_target_depth(test_depth)
        time.sleep(0.1)

    print("Yawing 180")
    yaw_start_time = time.time()
    while time.time() - yaw_start_time < 2 and button_state != "OFF":
        set_rc_channel_pwm(4, 2000)
        set_target_depth(test_depth)
        time.sleep(0.1)

    start_time = time.time()
    while time.time() - start_time < 5 and button_state != "OFF":
        set_desired_movement(x=300)
        set_target_depth(test_depth)
        time.sleep(0.1)

    set_desired_movement(x=0)
    print("Stopped movement")
    i += 1

    for _ in range(40):
        set_target_depth(test_depth)
        time.sleep(0.1)
    i += 1

# Function to handle button turned ON
def button_turned_on():
    global armed  # Use the global variable
    print("Button turned ON!")
    do_automation(test_depth)

# Function to handle button turned OFF
def button_turned_off():
    global armed  # Use the global variable
    if armed:
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        print("Button turned OFF!")
        print("DISARMED")

# Main loop for button press detection
try:
    while True:
        GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING)  # Wait for button press

        start = time.time()  # Record the start time of button press
        time.sleep(0.02)      # Debounce period

        while GPIO.input(BUTTON_PIN) == GPIO.LOW:
            time.sleep(0.01)

        length = time.time() - start

        if length >= BUTTON_PRESS_TIME:
            if not armed:
                print("armed")  # Print "armed" on the console for a long button press
                armed = True
                button_turned_on()
            else:
                print("disarmed")  # Print "disarmed" on the console for a long button press
                armed = False
                button_turned_off()

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
