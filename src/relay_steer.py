import RPi.GPIO as GPIO
import time

"""
Remember to use sudo when running script, for the raspberry pi to be able to access GPIO pins
"""

# Set the GPIO mode
GPIO.setmode(GPIO.BOARD)

# Set the pin number you're using for the relay
relay_pin = 7

# Setup the GPIO pin as an output
GPIO.setup(relay_pin, GPIO.OUT)

try:
    while True:
        # Turn on the relay
        GPIO.output(relay_pin, GPIO.HIGH)
        print("Relay is ON")
        time.sleep(1)  # Wait for 1 second

        # Turn off the relay
        GPIO.output(relay_pin, GPIO.LOW)
        print("Relay is OFF")
        time.sleep(1)  # Wait for 1 second

except KeyboardInterrupt:
    # Clean up GPIO settings
    GPIO.cleanup()
