from i2c import *


# Bme280 doesn't support auto-incrementing in writing mode!
class BME280:
    # CONSTANTS
    _ADDR = 0x77  # or 0x78!

    _ID_REG = 0xD0
    _TEMP_REG = [0xFA, 0xFB, 0xFC]
    _HUM_REG = [0xFD, 0xFE]
    _PRESS_REG = [0xF7, 0xF8, 0xF9]
    _RESET_REG = 0xE0
    _RESET = 0xB6
    _CTRL_HUM = 0xF2
    _STATUS = 0xF3
    _CTRL_MEAS = 0xF4
    _CONFIG = 0xF5

    dict_meas = {"temperature": None,   # [DegC]
                 "humidity": None,      # [%]
                 "pressure": None       # [hPa]
                 }

    _t_fine = None

    def __init__(self, i2c: I2C):
        self._i2c = i2c

    def read_id(self):
        self._i2c.write_byte(self._ADDR, self._ID_REG)
        dev_id = self._i2c.read(self._ADDR, 1)
        return dev_id == 0x60  # 0x60 is supposed value in this register acc to datasheet

    def reset_device(self):
        self._i2c.write_to_reg(self._ADDR, self._RESET_REG, self._RESET)

    def measure(self):
        # returns dictionary with all measurements
        registers = self._burst_read()
        temp_reg = registers[3:6]
        press_reg = registers[:3]
        hum_reg = registers[6:]

        raw_t = (temp_reg[0] * pow(2, 16) + temp_reg[1] * pow(2, 8) + temp_reg[2]) >> 4
        raw_p = (press_reg[0] * pow(2, 16) + press_reg[1] * pow(2, 8) + press_reg[2]) >> 4
        raw_h = hum_reg[0] * pow(2, 16) + hum_reg[1]

        trimming_param = self._read_trimming_param()

        self.dict_meas["temperature"] = self._compensate_temp(raw_t, trimming_param[:3])
        self.dict_meas["pressure"] = self._compensate_press(raw_p, trimming_param[3:12])
        self.dict_meas["humidity"] = self._compensate_hum(raw_h, trimming_param[12:])

        return self.dict_meas

    def _burst_read(self):
        # returns content of registers from 0xF7 to 0xFE
        self._i2c.write(self._ADDR, [0xF7])
        buf = self._i2c.read(self._ADDR, 8)
        return buf

    def _read_trimming_param(self):
        # returns content of trimming registers from 0x88 to 0xA1 and from 0xE1 to 0xE6
        self._i2c.write(self._ADDR, [0x88])
        buf = self._i2c.read(self._ADDR, 25)

        self._i2c.write(self._ADDR, [0xE1])
        buf.append(self._i2c.read(self._ADDR, 6))

        trim_reg = []
        for i in range(12):
            trim_reg[i] = buf[2*i] + buf[2*i + 1] * pow(2, 8)
        trim_reg[12] = buf[24]
        trim_reg[13] = buf[25] + buf[26] * pow(2, 8)
        trim_reg[14] = buf[27]
        trim_reg[15] = buf[28] * pow(2, 4) + (buf[29] & 0x0F)
        trim_reg[16] = buf[30] * pow(2, 4) + (buf[29] >> 4)

        return trim_reg

    def _compensate_temp(self, adc_t, dig_t):
        # compensates and returns temperature in DegC, resolution is 0.01 DegC
        var1 = (adc_t / 16384.0 - dig_t[0] / 1024.0) * dig_t[1]
        var2 = ((adc_t / 131072.0 - dig_t[0] / 8192.0) * (adc_t / 131072.0 - dig_t[0] / 8192.0)) * dig_t[2]
        self._t_fine = var1 + var2
        t = (var1 + var2) / 5120.0
        return t

    def _compensate_press(self, adc_p, dig_p):
        # compensates and returns pressure in hPa
        var1 = self._t_fine/2.0 - 64000.0
        var2 = var1 * var1 * dig_p[5] / 32768.0
        var2 += var1 * dig_p[4] * 2.0
        var2 = (var2 / 4.0) + (dig_p[3] * 65536.0)
        var1 = (dig_p[2] * var1 * var1 / 524288.0 + dig_p[1] * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * dig_p[0]
        if var1 == 0:
            return 0  # avoid exception caused by division by zero
        p = 1048576.0 - adc_p
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = dig_p[8] * p * p / 2147483648.0
        var2 = p * dig_p[7] / 32768.0
        p = p + (var1 + var2 + dig_p[6]) / 16.0
        return p/100.0

    def _compensate_hum(self, adc_h, dig_h):
        # compensates and returns humidity in %RH
        var1 = self._t_fine - 76800.0
        var1 = (adc_h - (dig_h[3] * 64.0 + dig_h[4] / 16384.0 * var1)) * \
               (dig_h[1] / 65536.0 * (1.0 + dig_h[5] / 67108864.0 * var1 * (1.0 + dig_h[2] / 67108864.0 * var1)))
        var1 *= (1.0 - dig_h[0] * var1 / 524288.0)
        if var1 > 100.0:
            var1 = 100.0
        if var1 < 0.0:
            var1 = 0.0
        return var1
