from sps30 import *
from i2c import *
# import time

if __name__ == '__main__':
    i2c = I2C(1)
    sps30 = SPS30(i2c)

    sps30.wake_up()
    print(sps30.read_device_serial())
    # print(sps30.read_firmware_ver())

    # sps30.start_measurement(sps30.MeasType.UINT_TYPE)
    # time.sleep(10)
    # sps30.read_measured_values()
    # meas = sps30.access_measured_values()
    # for i in meas:
    #     print(i+": "+meas[i])

