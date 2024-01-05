import RPi.GPIO as GPIO
import time

# GPIO pin and timing constants
BUTTON_PIN = 27                # Declare GPIO 27 for the button input
BUTTON_PRESS_TIME = 3       # Button press state time in seconds

# Set up GPIO configuration
GPIO.setmode(GPIO.BCM)          # Use BCM GPIO numbering
GPIO.setwarnings(False)         # Disable GPIO warnings
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Set up the button pin
time.sleep(1)  # Allow system initialization time

armed = False  # Variable to track the armed state

# Main loop for button press detection
try:
    while True:
        GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING)  # Wait for button press

        start = time.time()  # Record the start time of button press
        time.sleep(0.02)      # Debounce period

        # Continuously check button state while pressed
        while GPIO.input(BUTTON_PIN) == GPIO.LOW:
            time.sleep(0.01)

        length = time.time() - start  # Calculate the duration of button press

        # Check if the button press duration is greater than or equal to the defined time
        if length >= BUTTON_PRESS_TIME:
            if not armed:
                print("armed")  # Print "armed" on the console for a long button press
                armed = True
            else:
                print("disarmed")  # Print "disarmed" on the console for a long button press
                armed = False

except KeyboardInterrupt:
    pass  # Handle Ctrl+C gracefully

finally:
    GPIO.cleanup()  # Clean up GPIO resources on script exit
