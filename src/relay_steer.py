import gpiozero 
import time
from gpiozero.pins.mock import MockFactory

RELAY_PIN = 4
# Set the pin factory explicitly to the mock factory for non-Raspberry Pi environments

gpiozero.Device.pin_factory = MockFactory()
relay = gpiozero.OutputDevice(RELAY_PIN, active_high=True, initial_value=False)

try:
    while True:
        relay.on()
        time.sleep(5)

        relay.off()
        time.sleep(5)
except KeyboardInterrupt:
    print("stop the circuit")
    relay.close()
