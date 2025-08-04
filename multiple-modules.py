
import time
from VL53L1X import VL53L1X

# Pin Configuration for WEMOS D1 Mini
SCL_PIN = 9
SDA_PIN = 8

# Since we're using multiple sensors, we need to control power to each one
MODULE_1_XSHUT_PIN = 0
MODULE_2_XSHUT_PIN = 1
MODULE_3_XSHUT_PIN = 2

# Define new addresses for each module
MODULE_1_NEW_ADDRESS = 0x30
MODULE_2_NEW_ADDRESS = 0x31
MODULE_3_NEW_ADDRESS = 0x32

#Instantiate all three sensors
module_1 = VL53L1X(SCL_PIN, SDA_PIN, xshut_pin=MODULE_1_XSHUT_PIN)
module_2 = VL53L1X(SCL_PIN, SDA_PIN, xshut_pin=MODULE_2_XSHUT_PIN)
module_3 = VL53L1X(SCL_PIN, SDA_PIN, xshut_pin=MODULE_3_XSHUT_PIN)

#Put other devices to sleep while we change addresses
module_1.sleep()
module_2.sleep()
module_3.sleep()

# Waking then setting a new address for each module, one at a time
module_1.wake()
module_1.changeI2CAddress(MODULE_1_NEW_ADDRESS)

module_2.wake()
module_2.changeI2CAddress(MODULE_2_NEW_ADDRESS)

module_3.wake()
module_3.changeI2CAddress(MODULE_3_NEW_ADDRESS)


while True:
    distance_1 = module_1.distance()
    distance_2 = module_2.distance()
    distance_3 = module_3.distance()
    print(f"Distances: Module 1: {distance_1} mm, Module 2: {distance_2} mm, Module 3: {distance_3} mm")
    time.sleep(1)
