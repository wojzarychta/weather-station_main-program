from i2c import *
import struct
from enum import Enum


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

    _NO_ERROR = 1
    _ARTICLE_CODE_ERROR = -1
    _SERIAL_NUMBER_ERROR = -2
    _AUTO_CLN_INTERVAL_ERROR = -3
    _DATA_READY_FLAG_ERROR = -4
    _MEASURED_VALUES_ERROR = -5
    _VERSION_ERROR = -5

    class MeasType(Enum):
        IEEE754_TYPE = 0
        UINT_TYPE = 1

    chosen_meas_type = -1

    dict_output = {"pm1p0": None,   # [ug/m3]
                   "pm2p5": None,   # [ug/m3]
                   "pm4p0": None,   # [ug/m3]
                   "pm10p0": None,  # [ug/m3]
                   "nc0p5": None,   # [#/cm3]
                   "nc1p0": None,   # [#/cm3]
                   "nc2p5": None,   # [#/cm3]
                   "nc4p0": None,   # [#/cm3]
                   "nc10p0": None,  # [#/cm3]
                   "typical": None  # [um]- for float/[nm] - for uint
                   }

    def __init__(self, i2c: I2C):
        self._i2c = i2c

    @staticmethod
    def _calculateCRC(two_bytes):
        # checksum calculated according to sensiron's datasheet
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
        for i in range(2, len(result), 3):
            data = [result[i - 2], result[i - 1]]
            crc = result[i]
            if crc != self._calculateCRC(data):
                return False
        return True

    def read_device_serial(self):
        device_serial = []

        self._i2c.write(self._ADDR, self._READ_SERIAL_NUMBER)
        buf = self._i2c.read(self._ADDR, 48)

        if self._checkCRC(buf):
            for i in range(2, len(buf), 3):
                device_serial.append(chr(buf[i - 2]))
                device_serial.append(chr(buf[i - 1]))
            return str("".join(device_serial))
        else:
            return self._SERIAL_NUMBER_ERROR

    def read_firmware_ver(self):
        self._i2c.write(self._ADDR, self._READ_VERSION)
        buf = self._i2c.read(self._ADDR, 3)

        if self._checkCRC(buf):
            return buf[0] + "." + buf[1]
        else:
            return self._VERSION_ERROR

    def start_measurement(self, out_format: MeasType):
        self.chosen_meas_type = out_format

        byte_arr = self._START_MEAS
        if out_format == self.MeasType.IEEE754_TYPE:
            byte_arr.append(0x03)
        elif out_format == self.MeasType.UINT_TYPE:
            byte_arr.append(0x05)

        byte_arr.append(0x00)

        crc = self._calculateCRC(byte_arr[2:4])
        byte_arr.append(crc)

        self._i2c.write(self._ADDR, byte_arr)

    def stop_measurement(self):
        self._i2c.write(self._ADDR, self._STOP_MEAS)

    def read_measured_values(self):
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
            self._calculate_sensor_values(byte_array)
            return self._NO_ERROR
        else:
            return self._MEASURED_VALUES_ERROR

    def _calculate_sensor_values(self, byte_arr):
        if self.chosen_meas_type == self.MeasType.IEEE754_TYPE:
            i = 4
            for d in self.dict_output:
                # value is in IEEE 754 (sign, exponent and mantissa) format which needs to be parsed before writing
                value = byte_arr[i] + byte_arr[i - 1] * pow(2, 8) + \
                        byte_arr[i - 3] * pow(2, 16) + byte_arr[i - 4] * pow(2, 24)
                self.dict_output[d] = self._parse_IEEE754_to_float(value)
                i += 6
        elif self.chosen_meas_type == self.MeasType.UINT_TYPE:
            i = 1
            for d in self.dict_output:
                value = byte_arr[i] + byte_arr[i - 1] * pow(2, 8)
                self.dict_output[d] = value
                i += 3

    @staticmethod
    def _parse_IEEE754_to_float(value):
        string_value = str(hex(value)).replace("0x", "")
        byte_value = bytes.fromhex(string_value)
        return struct.unpack('>f', byte_value)[0]

    def access_measured_values(self):
        # returns dictionary
        return self.dict_output

    def read_data_ready_flag(self):
        # check if data is ready to read in synchronous mode
        # if so, return 1, else 0 or DATA_READY_FLAG_ERROR in case of wrong CRC
        self._i2c.write(self._ADDR, self._READ_DATA_RDY_FLAG)
        byte_arr = self._i2c.read(self._ADDR, 3)
        if self._checkCRC(byte_arr):
            # flag is stored in first byte of the returned array
            return byte_arr[1]
        else:
            return self._DATA_READY_FLAG_ERROR

    def wake_up(self):
        # in order to wake send wake-up cmd 2 times
        for i in range(2):
            self._i2c.write(self._ADDR, self._WAKE_UP)

    def sleep(self):
        self._i2c.write(self._ADDR, self._SLEEP)

    def start_fan_cleaning(self):
        self._i2c.write(self._ADDR, self._CLEAN_FAN)

    def device_reset(self):
        self._i2c.write(self._ADDR, self._RESET)
