import RPi.GPIO as GPIO
import time

# Customize this GPIO pin to match your relay connection
RELAY_PIN = 7

# Set up the GPIO mode
GPIO.setmode(GPIO.BOARD)  
GPIO.setup(RELAY_PIN, GPIO.OUT)

GPIO.output(RELAY_PIN, False)

try:
    while True:
        GPIO.output(RELAY_PIN, GPIO.HIGH)
        print("relay on")
        time.sleep(1)

        GPIO.output(RELAY_PIN, GPIO.LOW)
        print("relay off")
        time.sleep(1)     

except KeyboardInterrupt:   
    # Important: Clean up GPIO at the end
    GPIO.cleanup()
