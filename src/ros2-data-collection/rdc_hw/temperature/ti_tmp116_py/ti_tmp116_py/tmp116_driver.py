"""
TMP116 temperature sensor driver (I2C interface)

Hardware:
    Interface : I2C (bus 1, address 0x48)
    Library   : smbus2
    Datasheet : https://www.ti.com/lit/ds/symlink/tmp116.pdf
"""

from smbus2 import SMBus

I2C_BUS_ID   = 1
SENSOR_ADDR  = 0x48
TEMP_RESULT  = 0x00
CONFIGURATION = 0x01
TEMP_RESOLUTION = 0.0078125  # °C per LSB


class TMP116Driver:
    def __init__(self, bus_id: int = I2C_BUS_ID, address: int = SENSOR_ADDR):
        self.bus_id  = bus_id
        self.address = address
        self._configured = False

    def _configure(self, bus: SMBus) -> None:
        bus.write_i2c_block_data(self.address, CONFIGURATION, [0x02, 0x00])
        import time; time.sleep(0.1)
        self._configured = True

    def read_temperature(self) -> float:
        with SMBus(self.bus_id) as bus:
            if not self._configured:
                self._configure(bus)
            data = bus.read_i2c_block_data(self.address, TEMP_RESULT, 2)
            raw = (data[0] << 8) | data[1]
            if raw == 0x8000:
                raise RuntimeError('TMP116: data invalid flag set')
            if raw > 32767:
                raw -= 65536
            return raw * TEMP_RESOLUTION
