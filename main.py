# VL53L1X Distance Sensor - With Optional IRQ Pin
import time
from VL53L1X import VL53L1X

# Pin Configuration
SCL_PIN = 9
SDA_PIN = 8
XSHUT_PIN = 0 # optional - for power control
IRQ_PIN = 1 # optional - for interrupt-driven measurements 

#If you only have one module, can just tie XSHUT to high, and not use it here
sensor = VL53L1X(SCL_PIN, SDA_PIN)

# If using multiple sensors, you can use the XSHUT pin to control power to each
#sensor = VL53L1X(SCL_PIN, SDA_PIN, xshut_pin=XSHUT_PIN)

# For better reliability, use the IRQ pin (The module's GPIO1) to trigger measurements exactly when data is ready
#sensor = VL53L1X(SCL_PIN, SDA_PIN, irq_pin=IRQ_PIN, xshut_pin=XSHUT_PIN)

print("VL53L1X ready!")

while True:
    distance = sensor.distance()
    print(f"Distance: {distance} mm")
    time.sleep(1)
