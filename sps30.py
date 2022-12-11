from i2c import *
import struct
from enum import Enum
from time import sleep


class SPS30:
    # CONSTANTS
    _ADDR = 0x69

    _START_MEAS = [0x00, 0x10]
    _STOP_MEAS = [0x01, 0x04]
    _READ_DATA_RDY_FLAG = [0x02, 0x02]
    _READ_VALUES = [0x03, 0x00]
    _SLEEP = [0x10, 0x01]
    _WAKE_UP = [0x11, 0x03]
    _CLEAN_FAN = [0x56, 0x07]
    _RESET = [0xD3, 0x04]
    _READ_PRODUCT_TYPE = [0xD0, 0x02]
    _READ_SERIAL_NUMBER = [0xD0, 0x33]
    _READ_VERSION = [0xD1, 0x00]

    MEASUREMENT_ERROR = -1
    _NO_ERROR = 1
    _ARTICLE_CODE_ERROR = -7
    _SERIAL_NUMBER_ERROR = -2
    _AUTO_CLN_INTERVAL_ERROR = -3
    _DATA_READY_FLAG_ERROR = -4
    _MEASURED_VALUES_ERROR = -5
    _VERSION_ERROR = -6

    class MeasType(Enum):
        IEEE754_TYPE = 0
        UINT_TYPE = 1

    chosen_meas_type = -1

    dict_output = {"mass concentration PM1.0 [ug/m3]": 0,
                   "mass concentration PM2.5 [ug/m3]": 0,
                   "mass concentration PM4 [ug/m3]": 0,
                   "mass concentration PM10 [ug/m3]": 0,
                   "number concentration PM0.5 [#/cm3]": 0,
                   "number concentration PM1.0 [#/cm3]": 0,
                   "number concentration PM2.5 [#/cm3]": 0,
                   "number concentration PM4 [#/cm3]": 0,
                   "number concentration PM10 [#/cm3]": 0,
                   "typical particle size": 0  # [um]- for float/[nm] - for uint
                   }

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
        function checks if received information is correct and crc recived is correct
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
        :return: device's serial number in hex as string
        """
        device_serial = []

        self._i2c.write(self._ADDR, self._READ_SERIAL_NUMBER)
        buf = self._i2c.read(self._ADDR, 48)

        if self._checkCRC(buf):
            for i in range(2, len(buf), 3):
                device_serial.append(chr(buf[i - 2]))
                device_serial.append(chr(buf[i - 1]))
            return str("".join(device_serial))
        else:
            self._wrong_crc_exception_handler()

    def read_firmware_ver(self):
        """
        function reading device's firmware version
        :return: number of version
        """
        self._i2c.write(self._ADDR, self._READ_VERSION)
        buf = self._i2c.read(self._ADDR, 3)

        if self._checkCRC(buf):
            return buf[0] + "." + buf[1]
        else:
            return self._VERSION_ERROR

    def _start_measurement(self, out_format=MeasType.IEEE754_TYPE):
        """
        function which starts measurement cycle
        :param out_format: format of measurement (int or float); default float
        :return: none
        """
        self._wake_up()  # firstly wake up device from idle mode

        self.chosen_meas_type = out_format

        byte_arr = self._START_MEAS
        if out_format == self.MeasType.IEEE754_TYPE:
            byte_arr.append(0x03)
        elif out_format == self.MeasType.UINT_TYPE:
            byte_arr.append(0x05)

        byte_arr.append(0x00)  # dummy byte

        crc = self._calculateCRC(byte_arr[2:4])
        byte_arr.append(crc)

        self._i2c.write(self._ADDR, byte_arr)

    def _stop_measurement(self):
        """
        function which stops measurements and puts device in idle mode
        :return: none
        """
        self._i2c.write(self._ADDR, self._STOP_MEAS)

    def _read_measured_values(self) -> dict[str, float | int]:
        """
        function to read single measurement from device
        :return: dictionary with measurements
        """
        if self.chosen_meas_type == -1:
            raise Exception("Trying to read values before start of measurement")

        self._i2c.write(self._ADDR, self._READ_VALUES)

        bytes_to_read = 0
        if self.chosen_meas_type == self.MeasType.IEEE754_TYPE:
            bytes_to_read = 60
        elif self.chosen_meas_type == self.MeasType.UINT_TYPE:
            bytes_to_read = 30
        byte_array = self._i2c.read(self._ADDR, bytes_to_read)

        if self._checkCRC(byte_array):
            measured_values = self._calculate_sensor_values(byte_array)
            return measured_values
        else:  # handling event of communication error
            self._wrong_crc_exception_handler()

    def _calculate_sensor_values(self, byte_arr) -> dict[str, float]:
        """
        function which calculates actual values of measurement based on bytes received from device
        :param byte_arr: array of bytes which come from device after measurement
        :return: decoded measurement values as dictionary
        """
        values = dict(self.dict_output)

        if self.chosen_meas_type == self.MeasType.IEEE754_TYPE:
            i = 4
            for d in values:
                # value is in IEEE 754 (sign, exponent and mantissa) format which needs to be parsed before writing
                value = byte_arr[i] + byte_arr[i - 1] * pow(2, 8) + \
                        byte_arr[i - 3] * pow(2, 16) + byte_arr[i - 4] * pow(2, 24)
                values[d] = self._parse_IEEE754_to_float(value)
                i += 6
        elif self.chosen_meas_type == self.MeasType.UINT_TYPE:
            i = 1
            for d in values:
                value = byte_arr[i] + byte_arr[i - 1] * pow(2, 8)
                values[d] = value
                i += 3

        return values

    def _wrong_crc_exception_handler(self):
        """
        function which handles event of receiving wrong CRC, which is a transmission error
        :raises Exception: Communication error: Wrong CRC
        :return: none
        """
        self.device_reset()  # reset device before raising error
        raise Exception("Communication error: Wrong CRC")

    @staticmethod
    def _parse_IEEE754_to_float(value) -> float:
        """
        function which parses number from IEEE754 to float
        :param value: number in IEEE754 format
        :return: value as float
        """
        string_value = str(hex(value)).replace("0x", "")
        byte_value = bytes.fromhex(string_value)
        return struct.unpack('>f', byte_value)[0]

    def measure_pm(self, out_format=MeasType.IEEE754_TYPE):
        """
        function to measure particle matters implemented according to sps30 datasheet:
        - after waking up device wait 30 sec
        - then, for 30 sec read values once a sec
        - take average from above 30 measurements
        - put device to sleep
        :param out_format:
        :return: dictionary with correct measurements
        """
        self._start_measurement(out_format)
        sleep(30)
        measurements = dict(self.dict_output)
        for i in range(30):  # get 30 measurements to calculate avg
            while not self._read_data_ready_flag():  # wait for next values (approx. 1 sec)
                pass
            readout = self._read_measured_values()
            for m in measurements:
                measurements[m] += readout[m]
        # calculate average
        if self.chosen_meas_type == self.MeasType.IEEE754_TYPE:
            for i in measurements:
                measurements[i] /= 30
        elif self.chosen_meas_type == self.MeasType.UINT_TYPE:
            for i in measurements:
                measurements[i] //= 30

        self._stop_measurement()
        return measurements

    def measure_pm_and_print(self, out_format=MeasType.IEEE754_TYPE):
        """
        starts measurement and prints dictionary with measured values
        :return: none
        """
        meas_dict = self.measure_pm(out_format)
        for i in meas_dict:
            if i == "typical particle size":
                if self.chosen_meas_type == self.MeasType.IEEE754_TYPE:
                    print(i + " [um]" + ": " + f'{meas_dict[i]:.3f}')
                elif self.chosen_meas_type == self.MeasType.UINT_TYPE:
                    print(i + " [nm]" + ": " + f'{meas_dict[i]:.3f}')
            else:
                print(i + ": " + f'{meas_dict[i]:.3f}')

    def _read_data_ready_flag(self):
        """
        check if data is ready to read in synchronous mode
        if so, return 1, else 0 or DATA_READY_FLAG_ERROR in case of wrong CRC
        :return: True if flag is ready, False otherwise
        """
        self._i2c.write(self._ADDR, self._READ_DATA_RDY_FLAG)
        byte_arr = self._i2c.read(self._ADDR, 3)
        if self._checkCRC(byte_arr):
            # flag is stored in first byte of the returned array
            return byte_arr[1] == 0x01
        else:
            self._wrong_crc_exception_handler()

    def _wake_up(self):
        """
        wakes up device from idle mode
        in order to wake send wake-up command 2 times
        :return: none
        """
        for i in range(2):
            self._i2c.write(self._ADDR, self._WAKE_UP)

    def sleep(self):
        """
        puts device in sleep mode
        :return: none
        """
        self._i2c.write(self._ADDR, self._SLEEP)

    def start_fan_cleaning(self):
        self._i2c.write(self._ADDR, self._CLEAN_FAN)

    def device_reset(self):
        """
        resets device; after reset device is in idle mode and needs to be woke up
        :return: none
        """
        self._i2c.write(self._ADDR, self._RESET)
