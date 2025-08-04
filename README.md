# VL53L1X MicroPython Library

A simple and reliable MicroPython library for the VL53L1X Time-of-Flight (ToF) laser distance sensor.

## Features

- 🎯 **Simple distance measurements** - Get distance readings in millimeters
- 🔧 **Multiple sensor support** - Use multiple VL53L1X sensors on the same I2C bus
- ⚡ **Interrupt support** - Optional IRQ pin for efficient data-ready detection
- 🔄 **Auto-recovery** - Built-in detection and recovery from sensor lockups
- 💤 **Power management** - Sleep/wake functionality using XSHUT pin
- 🌐 **Tested platforms** - Works on ESP8266 and ESP32 boards

## Tested Hardware

- **ESP8266**: Wemos D1 Mini
- **ESP32**: Lolin D1 Mini V4 (ESP32-C3)
- **Sensor**: VL53L1X ToF distance sensor modules

## Installation

1. Copy `VL53L1X.py` to your MicroPython device
2. Import and use in your code

## Quick Start

### Basic Usage (Single Sensor)

```python
import time
from VL53L1X import VL53L1X

# Initialize sensor (SCL=9, SDA=8)
sensor = VL53L1X(scl_pin=9, sda_pin=8)

print("VL53L1X ready!")

while True:
    distance = sensor.distance()
    if distance > 0:
        print(f"Distance: {distance} mm")
    else:
        print("Measurement failed")
    time.sleep(1)
```

### With IRQ Pin (Recommended)

```python
from VL53L1X import VL53L1X

# Use IRQ pin for better timing and efficiency
sensor = VL53L1X(scl_pin=9, sda_pin=8, irq_pin=1, xshut_pin=0)

while True:
    if sensor.data_ready():
        distance = sensor.distance()
        print(f"Distance: {distance} mm")
    time.sleep(0.01)  # Short delay
```

### Multiple Sensors

```python
import time
from VL53L1X import VL53L1X

# Pin configuration
SCL_PIN = 9
SDA_PIN = 8

# Each sensor needs its own XSHUT pin for address changing
sensor1 = VL53L1X(SCL_PIN, SDA_PIN, xshut_pin=0)
sensor2 = VL53L1X(SCL_PIN, SDA_PIN, xshut_pin=1)
sensor3 = VL53L1X(SCL_PIN, SDA_PIN, xshut_pin=2)

# Put sensors to sleep before changing addresses
sensor1.sleep()
sensor2.sleep()
sensor3.sleep()

# Wake and set unique addresses one at a time
sensor1.wake()
sensor1.changeI2CAddress(0x30)

sensor2.wake()
sensor2.changeI2CAddress(0x31)

sensor3.wake()
sensor3.changeI2CAddress(0x32)

print("All sensors configured!")

while True:
    dist1 = sensor1.distance()
    dist2 = sensor2.distance()
    dist3 = sensor3.distance()
    print(f"Distances: {dist1}mm, {dist2}mm, {dist3}mm")
    time.sleep(1)
```

## Pin Connections

### Minimal Setup (Single Sensor)
```
VL53L1X → ESP32/ESP8266
VCC     → 3.3V
GND     → GND
SCL     → GPIO 9 (or your choice)
SDA     → GPIO 8 (or your choice)
```

### Full Setup (Multiple Sensors)
```
VL53L1X → ESP32/ESP8266
VCC     → 3.3V
GND     → GND
SCL     → GPIO 9 (shared)
SDA     → GPIO 8 (shared)
XSHUT   → GPIO 0,1,2... (unique per sensor)
GPIO1   → GPIO 1 (optional IRQ pin)
```

## API Reference

### Constructor
```python
VL53L1X(scl_pin, sda_pin, irq_pin=None, xshut_pin=None, address=0x29)
```

**Parameters:**
- `scl_pin`: I2C clock pin number
- `sda_pin`: I2C data pin number  
- `irq_pin`: Optional interrupt pin (connects to sensor's GPIO1)
- `xshut_pin`: Optional shutdown pin for power control
- `address`: I2C address (default 0x29)

### Methods

#### `distance(timeout_ms=1000)`
Get distance measurement in millimeters.
- **Returns**: Distance in mm, or -1 if measurement failed
- **Parameters**: `timeout_ms` - Maximum wait time for measurement

#### `data_ready()`
Check if new measurement data is available.
- **Returns**: `True` if data ready, `False` otherwise

#### `changeI2CAddress(new_address)`
Change the sensor's I2C address (requires XSHUT pin).
- **Parameters**: `new_address` - New I2C address (0x08-0x77)
- **Returns**: `True` if successful, `False` otherwise

#### `hardware_reset()`
Reset the sensor (automatically called if sensor gets stuck).

#### `sleep()` / `wake()`
Put sensor to sleep or wake it up (requires XSHUT pin).

## Troubleshooting

### Sensor Not Found
- Check wiring connections
- Verify I2C pins are correct
- Ensure 3.3V power supply

### Inconsistent Readings
- Use the IRQ pin for better timing
- Ensure stable power supply
- Check for electromagnetic interference

### Multiple Sensor Issues
- Each sensor must have its own XSHUT pin
- Change addresses one sensor at a time
- Verify each sensor works individually first

## Built-in Features

### Auto-Recovery
The library automatically detects when the sensor gets stuck (same reading 5 times) and performs a hardware reset.

### Error Handling
- Invalid measurements are automatically retried
- Timeout protection prevents infinite loops
- Clear error reporting for debugging

## Example Output

```
VL53L1X ready!
Distance: 245 mm
Distance: 187 mm
Distance: 156 mm
Distance: 201 mm
```

## Contributing

Found a bug or want to improve the library? Contributions are welcome!

## License

This library is provided as-is for educational and commercial use.

---


**Note**: This library was created because existing VL53L1X libraries for MicroPython were either incomplete or didn't work reliably. This implementation focuses on simplicity and reliability for practical applications.
