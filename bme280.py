from i2c import *
from time import sleep
import struct


def to_signed_short(val: int):
    """
    parses number to its signed representation
    :param val: parsed value
    :return: signed short number
    """
    return struct.unpack('<h', val.to_bytes(2, 'little'))[0]


class BME280:
    # CONSTANTS
    _ADDR = 0x76
    # REGISTERS
    _ID_REG = 0xD0
    _RESET_REG = 0xE0
    _RESET = 0xB6
    _CTRL_HUM = 0xF2
    _STATUS = 0xF3
    _CTRL_MEAS = 0xF4
    _CONFIG = 0xF5

    dig_t = [0] * 3
    dig_p = [0] * 9
    dig_h = [0] * 6

    dict_meas = {'t': 0.0,
                 'p': 0.0,
                 'h': 0.0
                 }

    _t_fine = None

    def __init__(self, i2c: I2C, address=0x76):
        """
        class for bme280 sensor service
        :param i2c: instance of i2c class
        :param address: i2c address of device (0x76 for SDO connected to GND; 0x77 when SDO connected to VDD)
        """
        self._i2c = i2c
        if address != 0x76 and address != 0x77:
            raise ValueError("Wrong address")
        self._ADDR = address

    def read_id(self):
        """
        reads device's id, which is supposed to be 0x60
        :return: True if id is correct (equal to 0x60)
        """
        self._i2c.write_byte(self._ADDR, self._ID_REG)
        dev_id = self._i2c.read(self._ADDR, 1)
        return dev_id[0] == 0x60

    def reset_device(self):
        """
        resets device
        :return: none
        """
        self._i2c.write_to_reg(self._ADDR, self._RESET_REG, self._RESET)

    def _get_compensation_parameters(self):
        """
        reads compensation parameters for compensation algorithms
        :return: none
        """
        self._i2c.write(self._ADDR, [0x88])
        buf = self._i2c.read(self._ADDR, 24)
        self._i2c.write(self._ADDR, [0xA1])
        buf.extend(self._i2c.read(self._ADDR, 1))
        self._i2c.write(self._ADDR, [0xE1])
        buf.extend(self._i2c.read(self._ADDR, 7))

        self.dig_t[0] = buf[0] + (buf[1] << 8)
        self.dig_t[1] = to_signed_short(buf[2] + (buf[3] << 8))
        self.dig_t[2] = to_signed_short(buf[4] + (buf[5] << 8))

        self.dig_p[0] = buf[6] + (buf[7] << 8)
        for i in range(4, 12):
            self.dig_p[i - 3] = to_signed_short(buf[2 * i] + (buf[2 * i + 1] << 8))

        self.dig_h[0] = buf[24]
        self.dig_h[1] = to_signed_short(buf[25] + buf[26] * pow(2, 8))
        self.dig_h[2] = buf[27]
        self.dig_h[3] = to_signed_short(buf[28] * pow(2, 4) + (buf[29] & 0x0F))
        self.dig_h[4] = to_signed_short(buf[30] * pow(2, 4) + (buf[29] >> 4))
        self.dig_h[5] = to_signed_short(buf[31])

    def access_measurements(self):
        """
        obtains measurements from registers
        returns temperature in DegC, pressure in hPa and humidity in %RH
        :return: dictionary with measurements
        """
        registers = self._burst_read()
        temp_reg = registers[3:6]
        press_reg = registers[:3]
        hum_reg = registers[6:]

        raw_t = (temp_reg[0] * pow(2, 16) + temp_reg[1] * pow(2, 8) + temp_reg[2]) >> 4
        raw_p = (press_reg[0] * pow(2, 16) + press_reg[1] * pow(2, 8) + press_reg[2]) >> 4
        raw_h = hum_reg[0] * pow(2, 8) + hum_reg[1]

        self.dict_meas['t'] = self._compensate_temp(raw_t)
        self.dict_meas['p'] = self._compensate_press(raw_p)
        self.dict_meas['h'] = self._compensate_hum(raw_h)

        return self.dict_meas

    def get_single_measurement(self):
        self.setup(mode='forced')
        # wait until measurement is made
        while self._is_measuring():  # wait for the end of measurement
            sleep(0.001)
        self.change_sensor_mode('sleep')
        return self.access_measurements()

    def print_single_measurement(self):
        """
        performs single measurement in forced mode and prints results
        :return: none
        """
        meas = self.get_single_measurement()
        print('{:<14}{:.2f} DegC\n{:<14}{:.2f} hPa\n{:<14}{:.2f}%'.format(
            "Temperature", meas['t'], "Pressure",  meas['p'], "Humidity",  meas['h']))

    def continuous_measurement(self, frequency=1):
        """
        performs measurement in normal mode and prints results
        :param frequency: frequency of measurements in Hz; available f = {2000, 100, 50, 16, 8, 4, 2, 1}
        :return: none
        """
        valid_f = {2000, 100, 50, 16, 8, 4, 2, 1}
        if frequency not in valid_f:
            raise ValueError("Wrong frequency")
        self.setup(mode='normal', standby=1000 / frequency)
        while True:
            meas = self.access_measurements()
            print('{:<14}{:.2f} DegC\n{:<14}{:.2f} hPa\n{:<14}{:.2f}%'.format(
                "Temperature", meas['t'], "Pressure",  meas['p'], "Humidity",  meas['h']))
            sleep(1 / frequency)

    def change_sensor_mode(self, mode='normal'):
        """
        executes transition of operating mode of sensor
        :param mode: operating mode of device: {'sleep', 'forced', 'normal'}
        :return: none
        """
        valid_modes = {'sleep': 0b00, 'forced': 0b01, 'normal': 0b11}
        if mode not in valid_modes:
            raise ValueError("Wrong mode")
        else:
            mode = valid_modes[mode]
        # read ctrl_meas register:
        self._i2c.write(self._ADDR, [self._CTRL_MEAS])
        reg = self._i2c.read(self._ADDR, 1)
        val = (reg[0] & 0xFC) | (mode & 0x03)  # writing mode to bits [1:0]
        self._i2c.write_to_reg(self._ADDR, self._CTRL_MEAS, val)

    def setup(self, mode='normal', t_oversampling=1, p_oversampling=1, h_oversampling=1, standby=1000):
        """
        setups device
        :param mode: operating mode of device: {'sleep', 'forced', 'normal'}
        :param t_oversampling: oversampling ratio of temperature data: {1, 2, 4, 8, 16}
        :param p_oversampling: oversampling ratio of pressure data: {1, 2, 4, 8, 16}
        :param h_oversampling: oversampling ratio of humidity data: {1, 2, 4, 8, 16}
        :param standby: inactive duration time in normal mode in ms: {0.5, 62.5, 125, 250, 500, 1000, 10, 20}
        :return: none
        """
        # check if device is connected
        if not self.read_id():
            raise RuntimeError("Unable to find bme280 on 0x{:02x}, IOError".format(self._ADDR))

        valid_modes = {'sleep': 0b00, 'forced': 0b01, 'normal': 0b11}
        if mode not in valid_modes:
            raise ValueError("Wrong mode")
        else:
            mode = valid_modes[mode]

        valid_osr = {1: 0b001, 2: 0b010, 4: 0b011, 8: 0b100, 16: 0b101}
        if t_oversampling not in valid_osr:
            raise ValueError("Wrong temperature oversampling ratio")
        else:
            t_oversampling = valid_osr[t_oversampling]
        if p_oversampling not in valid_osr:
            raise ValueError("Wrong pressure oversampling ratio")
        else:
            p_oversampling = valid_osr[p_oversampling]
        if h_oversampling not in valid_osr:
            raise ValueError("Wrong humidity oversampling ratio")
        else:
            h_oversampling = valid_osr[h_oversampling]

        valid_standby = {0.5: 0b000, 62.5: 0b001, 125: 0b010, 250: 0b011, 500: 0b100, 1000: 0b101, 10: 0b110, 20: 0b111}
        if standby not in valid_standby:
            raise ValueError("Wrong standby time")
        else:
            standby = valid_standby[standby]

        ctrl_hum = h_oversampling
        ctrl_meas = (t_oversampling << 5) + (p_oversampling << 2) + mode
        config = standby << 5  # turns filter off and doesn't enable 3-wire SPI

        # write values to registers
        self._i2c.write_to_reg(self._ADDR, self._CTRL_HUM, ctrl_hum)
        self._i2c.write_to_reg(self._ADDR, self._CTRL_MEAS, ctrl_meas)
        self._i2c.write_to_reg(self._ADDR, self._CONFIG, config)

        # read values for compensation algorithms
        self._get_compensation_parameters()

    def _is_measuring(self):
        """
        checks if device is currently measuring
        :return: True whenever a conversion is running
        """
        self._i2c.write(self._ADDR, [self._STATUS])
        buf = self._i2c.read(self._ADDR, 1)
        return buf[0] & 1 << 3  # 3rd bit is set to 1 while conversion is running

    def _burst_read(self):
        """
        reads content of registers from 0xF7 to 0xFE
        :return: content of registers from 0xF7 to 0xFE as list
        """
        self._i2c.write(self._ADDR, [0xF7])
        buf = self._i2c.read(self._ADDR, 8)
        return buf

    def get_temperature(self):
        return self.get_single_measurement()['t']

    def get_humidity(self):
        return self.self.get_single_measurement()['h']

    def get_pressure(self):
        return self.self.get_single_measurement()['p']

    def _compensate_temp(self, adc_t) -> float:
        """
        compensates and returns temperature in DegC, resolution is 0.01 DegC
        :param adc_t: adc output value
        :return: temperature in DegC
        """
        var1 = (adc_t / 16384.0 - self.dig_t[0] / 1024.0) * self.dig_t[1]
        var2 = ((adc_t / 131072.0 - self.dig_t[0] / 8192.0) * (adc_t / 131072.0 - self.dig_t[0] / 8192.0)) * self.dig_t[
            2]
        self._t_fine = var1 + var2
        t = (var1 + var2) / 5120.0
        return t

    def _compensate_press(self, adc_p) -> float:
        """
        compensates and returns pressure in hPa
        :param adc_p: adc output value
        :return: pressure in hPa
        """
        var1 = self._t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self.dig_p[5] / 32768.0
        var2 += var1 * self.dig_p[4] * 2.0
        var2 = (var2 / 4.0) + (self.dig_p[3] * 65536.0)
        var1 = (self.dig_p[2] * var1 * var1 / 524288.0 + self.dig_p[1] * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_p[0]
        if var1 == 0:
            return 0  # avoid exception caused by division by zero
        p = 1048576.0 - adc_p
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = self.dig_p[8] * p * p / 2147483648.0
        var2 = p * self.dig_p[7] / 32768.0
        p = p + (var1 + var2 + self.dig_p[6]) / 16.0
        return p / 100

    def _compensate_hum(self, adc_h) -> float:
        """
        compensates and returns humidity in %RH
        :param adc_h: adc output value
        :return: relative humidity in %
        """
        var1 = self._t_fine - 76800.0
        var1 = (adc_h - (self.dig_h[3] * 64.0 + self.dig_h[4] / 16384.0 * var1)) * \
               (self.dig_h[1] / 65536.0 * (
                           1.0 + self.dig_h[5] / 67108864.0 * var1 * (1.0 + self.dig_h[2] / 67108864.0 * var1)))
        var1 *= (1.0 - self.dig_h[0] * var1 / 524288.0)
        if var1 > 100.0:
            var1 = 100.0
        if var1 < 0.0:
            var1 = 0.0
        return var1
