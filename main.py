from sps30 import *
from i2c import *

if __name__ == '__main__':
    bus_number = 0

    i2c = I2C(bus_number)
    sps30 = SPS30(i2c)

    sps30.measure_pm_and_print()
