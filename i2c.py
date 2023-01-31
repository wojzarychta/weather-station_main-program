from smbus2 import *


def _bytes_to_int(num_of_bytes):
    result = 0
    for b in num_of_bytes:
        result = result * 256 + int(b)
    return result


class I2C:
    """
    class for I2C communication
    enables to write to and read from I2C (TWI) interface
    """

    def __init__(self, port):
        self.bus = SMBus(port)

    def write(self, address: int, buf: list):
        """
        writes data to i2c device
        :param address: i2c address of device
        :param buf: data to send to device as list of bytes
        :return: none
        """
        w = i2c_msg.write(address, buf)
        self.bus.i2c_rdwr(w)

    def read(self, address: int, bytes_to_read: int):
        """
        reads data from i2c device
        :param address: i2c address of device
        :param bytes_to_read: number of bytes to be read
        :return: list of read bytes; length of list is exactly bytes_to_read
        """
        result = []
        r = i2c_msg.read(address, bytes_to_read)
        self.bus.i2c_rdwr(r)
        for i in range(r.len):
            result.append(_bytes_to_int(r.buf[i]))
        return result

    def write_to_reg(self, address: int, reg: int, value: int):
        """
        writes data to i2c device to certain register
        :param address: i2c address of device
        :param reg: address of register
        :param value: value to write to register
        :return:
        """
        self.bus.write_byte_data(address, reg, value)

    def write_byte(self, address: int, value: int):
        """
        writes single byte to i2c device
        :param address: i2c address of device
        :param value: value to write to register
        :return:
        """
        self.bus.write_byte(address, value)
