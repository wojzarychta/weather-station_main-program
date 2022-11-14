from i2c import *
import struct
from enum import Enum

class SPS30():
    # CONSTANTS
    _ADDR = 0x69

    _START_MEAS = [0x00, 0x10]
    _STOP_MEAS = [0x01, 0x04]
    _READ_DATA_RDY_FLAG = [0x02, 0x02]
    _READ_VALUES = [0x03, 0x00]
    _SLEEP = [0x10, 0x01]
    _WAKE_UP = [0x11, 0x03]
    _CLEAR_FAN = [0x56, 0x07]
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

    class meas_type(Enum):
        IEEE754_TYPE = 0
        UINT_TYPE = 1

    chosen_meas_type = -1

    dict_output = {"pm1p0": None,  # [ug/m3]
                        "pm2p5": None,  # [ug/m3]
                        "pm4p0": None,  # [ug/m3]
                        "pm10p0": None,  # [ug/m3]
                        "nc0p5": None,  # [#/cm3]
                        "nc1p0": None,  # [#/cm3]
                        "nc2p5": None,  # [#/cm3]
                        "nc4p0": None,  # [#/cm3]
                        "nc10p0": None,  # [#/cm3]
                        "typical": None  # [um]- for float/[nm] - for uint
                   }


    def __init__(self, i2c: I2C):
        self._i2c = i2c


    def _calculateCRC(self, input):
        crc = 0xFF
        for i in range(0, 2):
            crc = crc ^ input[i]
            for j in range(8, 0, -1):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
        crc = crc & 0x0000FF
        return crc


    def _checkCRC(self, result):
        for i in range(2, len(result), 3):
            data = []
            data.append(result[i - 2])
            data.append(result[i - 1])

            crc = result[i]

            if crc != _calculateCRC(data):
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
            return self.SERIAL_NUMBER_ERROR


    def start_measurement(self, type: meas_type):
        self.chosen_meas_type = type

        byte_arr = self._START_MEAS
        match type:
            case self.meas_type.IEEE754_TYPE:
                byte_arr.append(0x03)
            case self.meas_type.UINT_TYPE:
                byte_arr.append(0x05)

        byte_arr.append(0x00)

        crc = calculateCRC(byte_arr[2:4])
        byte_arr.append(crc)

        self._i2c.write(self._ADDR, byte_arr)


    def stop_measurement(self):
        self._i2c.write(self._ADDR, self._STOP_MEAS)


    def read_measured_values(self):
        if self.chosen_meas_type == -1:
            raise Exception("Trying to read values before start of measurement")

        byte_array = []

        self._i2c.write(self._ADDR, self._READ_VALUES)

        bytes_to_read = 0
        match self.chosen_meas_type:
            case self.meas_type.IEEE754_TYPE:
                bytes_to_read = 60
            case self.meas_type.UINT_TYPE:
                bytes_to_read = 30
        byte_array = self._i2c.read(self._ADDR, bytes_to_read)

        if checkCRC(result):
            self._calculate_sensor_values(byte_array)
            return self.NO_ERROR
        else:
            return self.MEASURED_VALUES_ERROR


    def _calculate_sensor_values(self, input):
        match self.chosen_meas_type:
            case self.meas_type.IEEE754_TYPE:
                i = 4
                for d in self.dict_output:
                    value = input[i] + input[i - 1] * pow(2, 8) + input[i - 3] * pow(2, 16) + input[i - 4] * pow(2, 24) #value is in IEEE 754 (sign, exponent and mantissa) format which needs to be parsed before writing
                    self.dict_output[d] = self._parse_IEEE754_to_float(value)
                    i += 6
            case self.meas_type.UINT_TYPE:
                i = 1
                for d in self.dict_output:
                    value = input[i] + input[i - 1] * pow(2, 8)
                    self.dict_output[d] = value
                    i += 3


    def _parse_IEEE754_to_float(value):
        string_value = str(hex(value)).replace("0x", "")
        byte_value = bytes.fromhex(string_value)
        return struct.unpack('>f', byte_value)[0]


    def access_measuered_values(self):
        # returns values as dictionary
        return dict_output