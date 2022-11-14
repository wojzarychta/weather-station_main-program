# from smbus2 import *

#------SPG40---------
# SPG40_ADDR = 0x59
# SPG40_MEAS_RAW = 0x260F
# SPG40_MEAS_TEST = 0x280E
# SPG40_HEATER_OFF = 0x3615
# SPG40_MEAS_PARAM_WO_HUMIDITY = 0x8000A2666693

class I2C:

    def __init__(self, port):
        # self.bus = SMBus(port)
        pass

    def write(self, address: int, buf: list):
        # w = i2c_msg.write(address, buf)
        # self.bus.i2c_rdwr(w)
        pass

    @staticmethod
    def read(address: int, bytes_to_read: int):
        result = []
        # r = i2c_msg.read(address, bytes_to_read)
        # self.bus.i2c_rdwr(r)
        # for i in range(r.len):
        #     result.append(self._bytes_to_int(r.buf[i]))
        return result

    @staticmethod
    def _bytes_to_int(num_of_bytes):
        result = 0
        for b in num_of_bytes:
            result = result * 256 + int(b)
        return result
