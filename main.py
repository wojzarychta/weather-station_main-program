from sps30 import *
from i2c import *

if __name__ == '__main__':
    i2c = I2C(1)
    sps30 = SPS30(i2c)
    print(sps30.read_device_serial())
