'''!@file                   bno055.py
    @brief                  Class for the 9-axis BNO055 orientation sensor
    @details                
    @author                 Nathan Sullivan
    @date                   November 21, 2024
'''

from pyb import I2C
import struct

"""
after power on reset or soft reset, users need to wait at least
650ms before talking to BNO055 through I2C interface
"""


class BNO055:
    '''!@brief               Interface with the 9-axis sensor
        @details
    '''

    def __init__(self, i2c_bus):
        '''!@brief          Constructs BNO055 Object
            @details        Initializes the BNO055 for communication over i2c
            @param          i2c_bus
        '''
        self.i2c = I2C(i2c_bus, I2C.CONTROLLER, baudrate=100000)
        self.SENSOR_ADDRESS = 0x28
        self.OPR_MODE_REG = 0x3d

        self.CONFIG_MODE = 0x00
        self.IMU_MODE = 0x08
        self.COMPASS_MODE = 0x09
        self.M4G_MODE = 0x0A
        self.NDOF_FMC_OFF_MODE = 0x0B
        self.NDOF_MODE = 0x0C

        self.EUL_HEADING_MSB = 0x1b
        self.EUL_HEADING_LSB = 0x1a
        self.GYR_DATA_Z_MSB = 0x19
        self.GYR_DATA_Z_LSB = 0x18
        self.MAG_DATA_X_MSB = 0x0f
        self.MAG_DATA_X_LSB = 0x0e
        
        self.CALIB_STAT_REG = 0x35
        self.ACCEL_OFFSET_X_LSB = 0x55
        self.ACCEL_OFFSET_X_MSB = 0x56
        self.ACCEL_OFFSET_Y_LSB = 0x57
        self.ACCEL_OFFSET_Y_MSB = 0x58
        self.ACCEL_OFFSET_Z_LSB = 0x59
        self.ACCEL_OFFSET_Z_MSB = 0x5a
        self.MAG_OFFSET_X_LSB = 0x5b
        self.MAG_OFFSET_X_MSB = 0x5c
        self.MAG_OFFSET_Y_LSB = 0x5d
        self.MAG_OFFSET_Y_MSB = 0x5f
        self.MAG_OFFSET_Z_LSB = 0x60
        self.MAG_OFFSET_Z_MSB = 0x61
        self.GYRO_OFFSET_X_LSB = 0x62
        self.GYRO_OFFSET_X_MSB = 0x63
        self.GYRO_OFFSET_Y_LSB = 0x64
        self.GYRO_OFFSET_Y_MSB = 0x65
        self.GYRO_OFFSET_Z_LSB = 0x66
        self.GYRO_OFFSET_Z_MSB = 0x67
        self.ACCEL_RAD_LSB = 0x68
        self.ACCEL_RAD_MSB = 0x69
        self.MAG_RAD_LSB = 0x6a
        self.MAG_RAD_MSB = 0x6b

    def change_mode(self, mode):
        '''!@brief          Changes the sensor mode between config or one of the fusion modes
            @details
        '''
        if mode == self.CONFIG_MODE:
            self.mode = self.i2c.mem_write('\x00', self.SENSOR_ADDRESS, self.OPR_MODE_REG)
        if mode == self.IMU_MODE:
            self.mode = self.i2c.mem_write('\x08', self.SENSOR_ADDRESS, self.OPR_MODE_REG)
        if mode == self.COMPASS_MODE:
            self.mode = self.i2c.mem_write('\x09', self.SENSOR_ADDRESS, self.OPR_MODE_REG)
        if mode == self.M4G_MODE:
            self.mode = self.i2c.mem_write('\x0A', self.SENSOR_ADDRESS, self.OPR_MODE_REG)
        if mode == self.NDOF_FMC_OFF_MODE:
            self.mode = self.i2c.mem_write('\x0B', self.SENSOR_ADDRESS, self.OPR_MODE_REG)
        if mode == self.NDOF_MODE:
            self.mode = self.i2c.mem_write('\x0C', self.SENSOR_ADDRESS, self.OPR_MODE_REG)

    def retrieve_parse_cal(self):
        '''!@brief          retrieves a calibration update value
            @details        value of 255 is full calibration of all 3 sensors
            @return         Returns a decimal value between 0 and 255
        '''
        buf_cal = bytearray([0 for n in range(1)])
        self.i2c.mem_read(buf_cal, self.SENSOR_ADDRESS, self.CALIB_STAT_REG)
        unpack = struct.unpack('B', buf_cal)[0]
        return unpack

    def get_calibration(self):
        '''!@brief          retrieves calibration coefficients 
            @details        
            @return         Returns a buffer of 22 bytes for every coefficient value
        '''
        buf_get_cal = bytearray([0 for n in range(22)])
        for i in range(22):
            MEM_ADD = hex(self.ACCEL_OFFSET_X_LSB+i)
            self.i2c.mem_read(buf_get_cal, self.SENSOR_ADDRESS, MEM_ADD)
        return buf_get_cal

    def set_calibration(self, buffer):
        '''!@brief          Writes calibration coefficients
            @details        Allows for quick calibration of all 3 sensors
        '''
        for i in range(22):
            MEM_ADD = hex(self.ACCEL_OFFSET_X_LSB+i)
            self.i2c.mem_write(buffer, self.SENSOR_ADDRESS, MEM_ADD)

    def read_Euler(self):
        '''!@brief          Retrieves Euler Heading in degrees 
            @details
            @return         Returns a float between 0 and 360
        '''
        buf_Euler = bytearray([0 for n in range(2)])
        self.i2c.mem_read(buf_Euler, self.SENSOR_ADDRESS, self.EUL_HEADING_LSB)
        self.i2c.mem_read(buf_Euler, self.SENSOR_ADDRESS, self.EUL_HEADING_MSB)
        Heading = struct.unpack('>h', buf_Euler)[0]
        return Heading / 16.0

    def read_ang_vel(self):
        '''!@brief          Returns Z-axis Angular Velocity in degrees/second 
            @details
            @return         Returns a float
        '''
        buf_ang = bytearray([0 for n in range(2)])
        self.i2c.mem_read(buf_ang, self.SENSOR_ADDRESS, self.GYR_DATA_Z_LSB)
        self.i2c.mem_read(buf_ang, self.SENSOR_ADDRESS, self.GYR_DATA_Z_MSB)
        yaw = struct.unpack('>h', buf_ang)[0]
        return yaw / 900.0

    def home_status(self, value):
        '''!@brief          Determines a set Euler Heading for Home Position
            @details        Implemented for start and stop point for the robot driving in circle
            @return         Returns True if robot is at home and False if moving
        '''
        buf_home = bytearray([0 for n in range(2)])
        self.i2c.mem_read(buf_home, self.SENSOR_ADDRESS, self.EUL_HEADING_LSB)
        self.i2c.mem_read(buf_home, self.SENSOR_ADDRESS, self.EUL_HEADING_MSB)
        Heading = struct.unpack('>h', buf_home)[0]
        Heading = Heading / 16.0
        if value-3 <= Heading <= value+3:
            return True
        else:
            return False





