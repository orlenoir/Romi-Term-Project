<<<<<<< HEAD:Firmware/IMU_driver.py

from pyb import Pin
from utime import sleep_ms


class BNO055:
 
    ADDR_LOW = 0x28
    ADDR_HIGH = 0x29

    REG_CHIP_ID = 0x00
    CHIP_ID = 0xA0

    REG_OPR_MODE = 0x3D
    REG_PWR_MODE = 0x3E
    REG_SYS_TRIGGER = 0x3F
    REG_UNIT_SEL = 0x3B

    REG_CALIB_STAT = 0x35

    REG_EULER_H_LSB = 0x1A

    REG_GYRO_X_LSB = 0x14

    REG_OFFSET_START = 0x55
    OFFSET_LEN = 22

    MODE_CONFIG = 0x00
    MODE_ACCONLY = 0x01
    MODE_MAGONLY = 0x02
    MODE_GYRONLY = 0x03
    MODE_ACCMAG = 0x04
    MODE_ACCGYRO = 0x05
    MODE_MAGGYRO = 0x06
    MODE_AMG = 0x07
    MODE_IMUPLUS = 0x08    
    MODE_COMPASS = 0x09
    MODE_M4G = 0x0A
    MODE_NDOF_FMC_OFF = 0x0B
    MODE_NDOF = 0x0C       

    PWR_NORMAL = 0x00
    PWR_LOWPOWER = 0x01
    PWR_SUSPEND = 0x02

    def __init__(self, i2c, addr=ADDR_LOW, rst_pin=Pin.cpu.B5, do_reset=True):

        self.i2c = i2c
        self.addr = addr

        self.rst = None
        if rst_pin is not None:
            self.rst = Pin(rst_pin, mode=Pin.OUT_PP, value=1)

        if do_reset and self.rst is not None:
            self.hardware_reset()

        chip = self._r8(self.REG_CHIP_ID)
        if chip != self.CHIP_ID:
          
            sleep_ms(50)
            chip = self._r8(self.REG_CHIP_ID)
            if chip != self.CHIP_ID:
                raise OSError("BNO055 not found (CHIP_ID=0x{:02X})".format(chip))

        
        self.set_mode(self.MODE_CONFIG)


        self._w8(self.REG_PWR_MODE, self.PWR_NORMAL)
        sleep_ms(10)

    def hardware_reset(self):

        self.rst.low()
        sleep_ms(10)
        self.rst.high()

        sleep_ms(700)

    def set_mode(self, mode):

    
        self._w8(self.REG_OPR_MODE, self.MODE_CONFIG)
        sleep_ms(20)
        self._w8(self.REG_OPR_MODE, int(mode) & 0x0F)
        sleep_ms(20)

    def get_cal_status(self):

        b = self._r8(self.REG_CALIB_STAT)
        return {
            "sys":  (b >> 6) & 0x03,
            "gyro": (b >> 4) & 0x03,
            "accel": (b >> 2) & 0x03,
            "mag":  b & 0x03,
            "raw":  b
        }

    def read_cal_coeffs(self):


        prev = self._r8(self.REG_OPR_MODE)
        self.set_mode(self.MODE_CONFIG)
        data = self._rn(self.REG_OFFSET_START, self.OFFSET_LEN)
        self._w8(self.REG_OPR_MODE, prev)
        sleep_ms(10)
        return bytes(data)

    def write_cal_coeffs(self, coeff_bytes):

        if coeff_bytes is None or len(coeff_bytes) != self.OFFSET_LEN:
            raise ValueError("Expected {} bytes".format(self.OFFSET_LEN))

        prev = self._r8(self.REG_OPR_MODE)
        self.set_mode(self.MODE_CONFIG)
        self._wn(self.REG_OFFSET_START, coeff_bytes)
        self._w8(self.REG_OPR_MODE, prev)
        sleep_ms(10)

    def read_euler_deg(self):

        raw = self._rn(self.REG_EULER_H_LSB, 6)
        h = self._s16(raw[0], raw[1]) / 16.0
        r = self._s16(raw[2], raw[3]) / 16.0
        p = self._s16(raw[4], raw[5]) / 16.0
        return (h, r, p)

    def heading_deg(self):

        return self.read_euler_deg()[0]

    def read_gyro_dps(self):

        raw = self._rn(self.REG_GYRO_X_LSB, 6)
        gx = self._s16(raw[0], raw[1]) / 16.0
        gy = self._s16(raw[2], raw[3]) / 16.0
        gz = self._s16(raw[4], raw[5]) / 16.0
        return (gx, gy, gz)

    def yaw_rate_dps(self):

        return self.read_gyro_dps()[2]


    def _r8(self, reg):
        return self.i2c.mem_read(1, self.addr, reg)[0]

    def _w8(self, reg, val):
        self.i2c.mem_write(bytes([val & 0xFF]), self.addr, reg)

    def _rn(self, reg, n):
        return self.i2c.mem_read(n, self.addr, reg)

    def _wn(self, reg, data):

        self.i2c.mem_write(data, self.addr, reg)

    @staticmethod
    def _s16(lsb, msb):
        v = (msb << 8) | lsb
        if v & 0x8000:
            v -= 0x10000
=======

from pyb import Pin
from utime import sleep_ms


class BNO055:
 
    ADDR_LOW = 0x28
    ADDR_HIGH = 0x29

    REG_CHIP_ID = 0x00
    CHIP_ID = 0xA0

    REG_OPR_MODE = 0x3D
    REG_PWR_MODE = 0x3E
    REG_SYS_TRIGGER = 0x3F
    REG_UNIT_SEL = 0x3B

    REG_CALIB_STAT = 0x35

    REG_EULER_H_LSB = 0x1A

    REG_GYRO_X_LSB = 0x14

    REG_OFFSET_START = 0x55
    OFFSET_LEN = 22

    MODE_CONFIG = 0x00
    MODE_ACCONLY = 0x01
    MODE_MAGONLY = 0x02
    MODE_GYRONLY = 0x03
    MODE_ACCMAG = 0x04
    MODE_ACCGYRO = 0x05
    MODE_MAGGYRO = 0x06
    MODE_AMG = 0x07
    MODE_IMUPLUS = 0x08    
    MODE_COMPASS = 0x09
    MODE_M4G = 0x0A
    MODE_NDOF_FMC_OFF = 0x0B
    MODE_NDOF = 0x0C       

    PWR_NORMAL = 0x00
    PWR_LOWPOWER = 0x01
    PWR_SUSPEND = 0x02

    def __init__(self, i2c, addr=ADDR_LOW, rst_pin=Pin.cpu.B5, do_reset=True):

        self.i2c = i2c
        self.addr = addr

        self.rst = None
        if rst_pin is not None:
            self.rst = Pin(rst_pin, mode=Pin.OUT_PP, value=1)

        if do_reset and self.rst is not None:
            self.hardware_reset()

        chip = self._r8(self.REG_CHIP_ID)
        if chip != self.CHIP_ID:
          
            sleep_ms(50)
            chip = self._r8(self.REG_CHIP_ID)
            if chip != self.CHIP_ID:
                raise OSError("BNO055 not found (CHIP_ID=0x{:02X})".format(chip))

        
        self.set_mode(self.MODE_CONFIG)


        self._w8(self.REG_PWR_MODE, self.PWR_NORMAL)
        sleep_ms(10)

    def hardware_reset(self):

        self.rst.low()
        sleep_ms(10)
        self.rst.high()

        sleep_ms(700)

    def set_mode(self, mode):

    
        self._w8(self.REG_OPR_MODE, self.MODE_CONFIG)
        sleep_ms(20)
        self._w8(self.REG_OPR_MODE, int(mode) & 0x0F)
        sleep_ms(20)

    def get_cal_status(self):

        b = self._r8(self.REG_CALIB_STAT)
        return {
            "sys":  (b >> 6) & 0x03,
            "gyro": (b >> 4) & 0x03,
            "accel": (b >> 2) & 0x03,
            "mag":  b & 0x03,
            "raw":  b
        }

    def read_cal_coeffs(self):


        prev = self._r8(self.REG_OPR_MODE)
        self.set_mode(self.MODE_CONFIG)
        data = self._rn(self.REG_OFFSET_START, self.OFFSET_LEN)
        self._w8(self.REG_OPR_MODE, prev)
        sleep_ms(10)
        return bytes(data)

    def write_cal_coeffs(self, coeff_bytes):

        if coeff_bytes is None or len(coeff_bytes) != self.OFFSET_LEN:
            raise ValueError("Expected {} bytes".format(self.OFFSET_LEN))

        prev = self._r8(self.REG_OPR_MODE)
        self.set_mode(self.MODE_CONFIG)
        self._wn(self.REG_OFFSET_START, coeff_bytes)
        self._w8(self.REG_OPR_MODE, prev)
        sleep_ms(10)

    def read_euler_deg(self):

        raw = self._rn(self.REG_EULER_H_LSB, 6)
        h = self._s16(raw[0], raw[1]) / 16.0
        r = self._s16(raw[2], raw[3]) / 16.0
        p = self._s16(raw[4], raw[5]) / 16.0
        return (h, r, p)

    def heading_deg(self):

        return self.read_euler_deg()[0]

    def read_gyro_dps(self):

        raw = self._rn(self.REG_GYRO_X_LSB, 6)
        gx = self._s16(raw[0], raw[1]) / 16.0
        gy = self._s16(raw[2], raw[3]) / 16.0
        gz = self._s16(raw[4], raw[5]) / 16.0
        return (gx, gy, gz)

    def yaw_rate_dps(self):

        return self.read_gyro_dps()[2]


    def _r8(self, reg):
        return self.i2c.mem_read(1, self.addr, reg)[0]

    def _w8(self, reg, val):
        self.i2c.mem_write(bytes([val & 0xFF]), self.addr, reg)

    def _rn(self, reg, n):
        return self.i2c.mem_read(n, self.addr, reg)

    def _wn(self, reg, data):

        self.i2c.mem_write(data, self.addr, reg)

    @staticmethod
    def _s16(lsb, msb):
        v = (msb << 8) | lsb
        if v & 0x8000:
            v -= 0x10000
>>>>>>> ee3ec90 (update files, docs, reports):firmware/IMU_driver.py
        return v