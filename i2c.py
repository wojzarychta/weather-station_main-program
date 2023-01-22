from smbus2 import *


def _bytes_to_int(num_of_bytes):
    result = 0
    for b in num_of_bytes:
        result = result * 256 + int(b)
    return result


class I2C:

    def __init__(self, port):
        self.bus = SMBus(port)

    def write(self, address: int, buf: list):
        w = i2c_msg.write(address, buf)
        self.bus.i2c_rdwr(w)

    def read(self, address: int, bytes_to_read: int):
        result = []
        r = i2c_msg.read(address, bytes_to_read)
        self.bus.i2c_rdwr(r)
        for i in range(r.len):
            result.append(_bytes_to_int(r.buf[i]))
        return result

    def write_to_reg(self, address: int, reg: int, value: int):
        self.bus.write_byte_data(address, reg, value)

    def write_byte(self, address: int, value: int):
        self.bus.write_byte(address, value)
