from i2c import *
import math
import time


def calculate_absolute_humidity(rh: float, t: float):
    """
    function calculating absolute humidity based on RH [%] and temperature [DegC]
    :param rh: relative humidity in %; float
    :param t: temperature in DegC; float
    :return: absolute humidity in float
    """
    abs_hum = 216.7 * (rh / 100 * 6.112 * math.exp(17.62 * t / (243.12 + t)) / (273.15 + t))
    return abs_hum


def parse_float_to_fixed_point(f: float):
    """
    parses float to fixed-point 8.8bit number
    :param f: float
    :return: fixed-point 8.8bit number as a list of two bytes [MSB, LSB]
    """
    fixed = [int(f), int(f % 1 * 256)]
    return fixed


class SGP30:
    # CONSTANTS
    _ADDR = 0x58

    _IAQ_INIT = [0x20, 0x03]
    _MEASURE_IAQ = [0x20, 0x08]
    _GET_IAQ_BASELINE = [0x20, 0x15]
    _SET_IAQ_BASELINE = [0x20, 0x1e]
    _SET_ABSOLUTE_HUMIDITY = [0x20, 0x61]
    _GET_FEATURE_TEST = [0x20, 0x2f]
    _MEASURE_RAW = [0x20, 0x50]
    _GET_TVOC_INCEPTIVE_BASELINE = [0x20, 0xb3]
    _SET_TVOC_BASELINE = [0x20, 0x77]
    _READ_SERIAL_NUMBER = [0x36, 0x82]
    _GENERAL_CALL_ADDRESS = 0x00
    _RESET = 0x06

    measurement = {"TVOC": 0,
                   "CO2eq": 0
                   }

    measurement_ready = False

    def __init__(self, i2c: I2C):
        self._i2c = i2c

    @staticmethod
    def _calculateCRC(two_bytes):
        """
        function calculates crc accordingly to datasheet (polynomial = 0x31)
        :param two_bytes: list of two_bytes from which crc is to be calculated: list
        :return: crc value: int
        """
        # checksum calculated according to
        crc = 0xFF
        for i in range(0, 2):
            crc = crc ^ two_bytes[i]
            for j in range(8, 0, -1):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
        crc = crc & 0x0000FF
        return crc

    def _checkCRC(self, result):
        """
        function checks if received information is correct and crc received is correct
        :param result: list of received bytes, divisible by 3: list
        :return: boolean, True if information is free of errors
        """
        for i in range(2, len(result), 3):
            data = [result[i - 2], result[i - 1]]
            crc = result[i]
            if crc != self._calculateCRC(data):
                return False
        return True

    def read_device_serial(self):
        """
        function reading device's serial number
        :raises Exception: Communication error: Wrong CRC
        :return: device's serial number in hex as string
        """
        device_serial = 0

        self._i2c.write(self._ADDR, self._READ_SERIAL_NUMBER)
        buf = self._i2c.read(self._ADDR, 6)

        if self._checkCRC(buf):
            for i in range(len(buf)):
                if (i+1) % 3 == 0:
                    continue
                else:
                    device_serial = (device_serial << 8) + buf[i]
            return hex(device_serial)
        else:
            self._wrong_crc_exception_handler()

    def _wrong_crc_exception_handler(self):
        """
        function which handles event of receiving wrong CRC, which is a transmission error
        raises Exception: Communication error: Wrong CRC
        :return: none
        """
        # self.device_reset()  # reset device before raising error
        raise Exception("Communication error: Wrong CRC")

    def _start_measurement(self):
        """
        function which starts measurement cycle
        :return: none
        """
        self._i2c.write(self._ADDR, self._IAQ_INIT)
        time.sleep(0.01)

    def _single_measurement(self):
        """
        does one single measurement of air quality signals
        :return: list of measurement, [CO2eq, TVOC]
        """
        self._i2c.write(self._ADDR, self._MEASURE_IAQ)
        time.sleep(0.012)
        bytes_to_read = 6
        byte_array = self._i2c.read(self._ADDR, bytes_to_read)

        if self._checkCRC(byte_array):
            co2eq = byte_array[0] * pow(2, 8) + byte_array[1]
            tvoc = byte_array[3] * pow(2, 8) + byte_array[4]
            return [co2eq, tvoc]
        else:  # handling event of communication error
            self._wrong_crc_exception_handler()

    def _set_absolute_humidity(self, rh: float, t: float):
        """
        function sets absolute humidity in device for air quality signals compensation
        :param rh: relative humidity in %; float
        :param t: temperature in DegC; float
        :return: none
        """
        abs_hum = calculate_absolute_humidity(rh, t)
        abs_hum_fixed = parse_float_to_fixed_point(abs_hum)
        if abs_hum_fixed[0] > 255:
            abs_hum_fixed[0] = 255
        crc = self._calculateCRC(abs_hum_fixed)
        abs_hum_fixed.append(crc)

        self._i2c.write(self._ADDR, abs_hum_fixed)

    def measure(self) -> dict[str, int]:
        """
        function to measure air tvoc in ppb and CO2eq in ppm
        :return: dict with outputs
        """
        self._start_measurement()
        # self._set_absolute_humidity()  # optional
        start_time = time.time()
        while True:
            meas = self._single_measurement()
            if meas[0] != 400 and meas[1] != 0:  # during first 15sec sensor returns fixed values of 400 ppm CO2eq
                # and 0 ppb TVOC
                break
            time.sleep(1)
            while time.time() - start_time < 1:  # sleep for 1 s
                pass
            start_time = time.time()
        self.measurement["TVOC"] = meas[1]
        self.measurement["CO2eq"] = meas[0]
        self.measurement_ready = True
        self.print_measurement()  # pozniej usunac
        return self.measurement

    def print_measurement(self):
        if self.measurement_ready:
            print('{}{:05.2f} ppb\n{}{:05.2f} ppm'.format("TVOC:", self.measurement["TVOC"],
                                                          "CO2eq:", self.measurement["CO2eq"]))
        else:
            raise RuntimeWarning("There is no measurement to be printed")

    def soft_reset(self):
        """
        Resets sensors using “General Call” mode according to I2C-bus specification.
        It is important to understand that a reset generated in this way is not device specific.
        All devices on the same I2C bus that support the General Call mode will perform a reset.
        :return: none
        """
        self._i2c.write_byte(self._GENERAL_CALL_ADDRESS, self._RESET)

