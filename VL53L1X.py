# VL53L1X MicroPython Library
# Basic distance measurement functionality for VL53L1X ToF sensor

import time
from micropython import const
from machine import Pin, I2C

NUM_DUPE_READINGS_UNTIL_RESET = 5  # Number of duplicate readings before reset

# Essential register constants
_VL53L1X_IDENTIFICATION__MODEL_ID = const(0x010F)
_SYSTEM__INTERRUPT_CLEAR = const(0x0086)
_SYSTEM__MODE_START = const(0x0087)
_VL53L1X_RESULT__RANGE_STATUS = const(0x0089)
_VL53L1X_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = const(0x0096)
_VL53L1X_DEFAULT_DEVICE_ADDRESS = const(0x29)
_VL53L1X_FIRMWARE__SYSTEM_STATUS = const(0x00E5)
_I2C_SLAVE__DEVICE_ADDRESS = const(0x0001)

# Default configuration array (essential registers 0x2D to 0x87)
_VL53L1X_DEFAULT_CONFIGURATION = bytes([
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21, 0x00, 0x00, 0x05, 0x00,
    0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xcc, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08,
    0xb8, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00, 0x00, 0x02, 0xc7, 0xff,
    0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00
])


class VL53L1X:
    """VL53L1X distance sensor driver for MicroPython."""

    def __init__(self, scl_pin, sda_pin, irq_pin=None, xshut_pin=None, address=_VL53L1X_DEFAULT_DEVICE_ADDRESS):
        self.i2c = I2C(scl=Pin(scl_pin), sda=Pin(sda_pin))
        self.address = address
        self._buffer = bytearray(2)
        self._last_distance = None
        self._same_reading_count = 0
        
        #Set up the xshut_pin pin as output, and keep it high if provided
        self.xshut_pin = None
        if xshut_pin is not None:
            self.xshut_pin = Pin(xshut_pin, Pin.OUT)
            self.xshut_pin.value(1)

        #Set up the IRQ pin if provided
        self.irq_pin = None
        if irq_pin is not None:
            self.irq_pin = Pin(irq_pin, Pin.IN, Pin.PULL_UP)

        # Verify sensor is present
        model_id = self._read_u16(_VL53L1X_IDENTIFICATION__MODEL_ID)
        if model_id != 0xEACC:
            raise RuntimeError(f"VL53L1X not found. Got model ID: {hex(model_id)}")

        # Initialize sensor
        self._init_sensor()

    def _init_sensor(self):
        """Initialize the sensor with basic configuration."""
        # Wait for sensor boot
        for _ in range(100):
            if self._read_u8(_VL53L1X_FIRMWARE__SYSTEM_STATUS) != 0:
                break
            time.sleep_ms(2)
        
        # Write default configuration
        for i, val in enumerate(_VL53L1X_DEFAULT_CONFIGURATION):
            self._write_u8(0x2D + i, val)
        
        # Calibration sequence
        self._write_u8(_SYSTEM__MODE_START, 0x40)  # Start ranging
        time.sleep_ms(100)  # Wait for calibration
        self._write_u8(_SYSTEM__INTERRUPT_CLEAR, 0x01)  # Clear interrupt
        self._write_u8(_SYSTEM__MODE_START, 0x00)  # Stop ranging
        
        # Start normal operation
        self._write_u8(_SYSTEM__MODE_START, 0x40)

    def data_ready(self):
        """Check if new measurement data is ready using IRQ pin or polling."""
        if self.irq_pin:
            # Use hardware interrupt pin if available
            return not self.irq_pin.value()  # IRQ pin goes low when data ready
        else:
            # Fall back to polling method
            range_status = self._read_u8(_VL53L1X_RESULT__RANGE_STATUS)
            return (range_status & 0x1F) == 9

    def distance(self, timeout_ms=1000):
        """
        Get distance measurement in millimeters.
        Returns distance in mm, or -1 if measurement failed.
        """
        start_time = time.ticks_ms()
        
        while time.ticks_diff(time.ticks_ms(), start_time) < timeout_ms:
            # Check if measurement is ready
            range_status = self._read_u8(_VL53L1X_RESULT__RANGE_STATUS)
            raw_status = range_status & 0x1F
            
            # Status 9 indicates valid measurement
            if raw_status == 9:
                dist = self._read_u16(_VL53L1X_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0)
                
                # Simple stuck detection - same reading 5 times in a row
                if self._last_distance == dist:
                    self._same_reading_count += 1
                    if self._same_reading_count >= NUM_DUPE_READINGS_UNTIL_RESET:
                        print("Sensor stuck, performing hardware reset...")
                        self.hardware_reset()
                        self._same_reading_count = 0
                        time.sleep_ms(100)
                        continue
                else:
                    self._same_reading_count = 0
                
                self._last_distance = dist
                self._write_u8(_SYSTEM__INTERRUPT_CLEAR, 0x01)  # Clear for next measurement
                return dist
                
            elif raw_status != 0:
                # Invalid measurement, restart
                self._write_u8(_SYSTEM__INTERRUPT_CLEAR, 0x01)
                self._write_u8(_SYSTEM__MODE_START, 0x40)
                
            time.sleep_ms(10)
        
        return -1  # Timeout

    def hardware_reset(self):
        """Perform hardware reset using XSHUT pin if available, otherwise software reset."""
        if self.xshut_pin is not None:
            # Hardware reset using XSHUT pin
            self.xshut_pin.value(0)  # Pull XSHUT low to reset
            time.sleep_ms(850)  # Keep low for 10ms
            self.xshut_pin.value(1)  # Release reset
            time.sleep_ms(250)  # Wait for sensor to boot

            # Reinitialize sensor
            self._init_sensor()
            self._last_distance = None
            self._same_reading_count = 0

        else:
            # Fall back to software reset
            self._write_u8(_SYSTEM__MODE_START, 0x00)
            time.sleep_ms(50)
            
            # Clear any pending interrupts
            self._write_u8(_SYSTEM__INTERRUPT_CLEAR, 0x01)
            time.sleep_ms(10)
            
            # Restart with fresh initialization
            self._write_u8(_SYSTEM__MODE_START, 0x40)
            time.sleep_ms(100)

    def changeI2CAddress(self, new_address):
        """ Change the I2C address of the sensor - XSHUT pin must be connected to use this feature with multiple sensors."""
        
        if new_address < 0x08 or new_address > 0x77:
            print(f"Error: Address {hex(new_address)} out of valid range (0x08-0x77)")
            return False
            
        try:
            # Write new address to the device
            self._write_u8(_I2C_SLAVE__DEVICE_ADDRESS, new_address & 0x7F)
            time.sleep_ms(50)
            
            # Update our stored address
            old_address = self.address
            self.address = new_address
            
            # Verify the change worked by reading model ID at new address
            model_id = self._read_u16(_VL53L1X_IDENTIFICATION__MODEL_ID)
            if model_id == 0xEACC:
                print(f"Address changed from {hex(old_address)} to {hex(new_address)}")
                return True
            else:
                # Revert address if verification failed
                self.address = old_address
                print(f"Address change failed - verification read returned {hex(model_id)}")
                return False
                
        except Exception as e:
            print(f"Error changing address: {e}")
            return False


    def sleep(self):
        """Put the sensor into low-power sleep mode."""
        if self.xshut_pin is not None:
            self.xshut_pin.value(0)  # Pull XSHUT low to put sensor to sleep
            time.sleep_ms(10)
            print("Sensor put to sleep via XSHUT pin.")
        else:
            print("No XSHUT pin available, cannot put sensor to sleep.")
    
    def wake(self):
        """Wake the sensor from low-power sleep mode."""
        if self.xshut_pin is not None:
            self.xshut_pin.value(1)
            time.sleep_ms(10)  # Allow time for sensor to wake
            print("Sensor woken up via XSHUT pin.")
        else:
            print("No XSHUT pin available, cannot wake sensor.")

    
    def _write_u8(self, reg, val):
        """Write single byte to register."""
        self.i2c.writeto_mem(self.address, reg, bytes([val]), addrsize=16)

    def _read_u8(self, reg):
        """Read single byte from register."""
        return self.i2c.readfrom_mem(self.address, reg, 1, addrsize=16)[0]

    def _read_u16(self, reg):
        """Read 16-bit value from register."""
        self.i2c.readfrom_mem_into(self.address, reg, self._buffer, addrsize=16)
        return (self._buffer[0] << 8) | self._buffer[1]
