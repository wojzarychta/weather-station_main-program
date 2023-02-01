from sps30 import *
from i2c import *
from bme280 import *
from sgp30 import *
import serial
import threading
import time


def progress_bar(current, total, bar_length=30):
    fraction = current / total
    bar = int(fraction * bar_length) * '\u2588'
    padding = int(bar_length - len(bar)) * ' '
    ending = '\n' if current == total else '\r'
    print(f'Progress: |{bar}{padding}| {int(fraction * 100)}% done', end=ending)


def update_progress_bar(duration: int, bar_length=30):
    """
    updates progress bar lasting 'duration' seconds
    :param duration: time of process in sec
    :param bar_length: length of bar displayed in console
    :return: none
    """
    progress_bar(0, duration, bar_length)
    start_time = time.time()
    for i in range(1, duration+1):
        while time.time() - start_time < 1:  # sleep for 1 s
            pass
        start_time = time.time()
        progress_bar(i, duration, bar_length)


if __name__ == '__main__':
    # UART:
    serial_port_name = "/dev/ttyS1"
    port = serial.Serial(serial_port_name, baudrate=115200)
    # I2C
    bus_number = 0
    i2c = I2C(bus_number)
    # sensors:
    sps30 = SPS30(i2c)
    bme280 = BME280(i2c)
    sgp30 = SGP30(i2c)

    # initialize threads
    sps_thread = threading.Thread(target=sps30.measure)
    sgp_thread = threading.Thread(target=sgp30.measure)
    progress_bar_thread = threading.Thread(target=update_progress_bar, args=(60,))

    sps_thread.start()
    sgp_thread.start()
    progress_bar_thread.start()

    while sps_thread or sgp_thread or progress_bar_thread:  # wait for the end of measurements
        pass

    # collect data from STM32
    pass

    # print measurements
    bme280.print_single_measurement()
    sps30.print_measurement()
    sgp30.print_measurement()
