import RPi.GPIO as GPIO
import time
import keyboard
import automove


# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pin for the switch
switch_pin = 17

# Set up the switch as input with a pull-up resistor
GPIO.setup(switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Variables to track switch states
switch_state = "CENTER"

# Define functions for each switch state
def switch_turned_on():
    print("Switch turned ON!")
    automove.do_automation(test_depth)

    # Add code for what to do when the switch is turned on
    # For example, you can add GPIO outputs, start a process, etc.

def switch_turned_off():
    print("Switch turned OFF!")
    k_key_pressed = keyboard.is_pressed('k')
    automove.send_manual_control_inputs(k_key_pressed)
    # Add code for what to do when the switch is turned off
    # For example, you can stop a process, turn off GPIO outputs, etc.

def switch_in_center():
    print("Switch in the center position.")
    
    # Add code for what to do when the switch is in the center position

try:
    while True:
        # Read the current state of the switch
        current_switch_state = GPIO.input(switch_pin)

        # Check for a rising edge (transition from off to on)
        if current_switch_state == GPIO.LOW and switch_state != "ON":
            switch_turned_on()
            switch_state = "ON"

        # Check for a falling edge (transition from on to off)
        elif current_switch_state == GPIO.HIGH and switch_state != "OFF":
            switch_turned_off()
            switch_state = "OFF"

        # Check for the side state
        elif current_switch_state == GPIO.HIGH and switch_state != "CENTER":
            switch_in_center()
            switch_state = "CENTER"

        time.sleep(0.1)

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
