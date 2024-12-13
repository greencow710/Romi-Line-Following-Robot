# -*- coding: utf-8 -*-
"""
Created on Thu Dec  5 13:20:46 2024

@author: green
"""

from pyb import Pin, Timer, micros, delay
from time import sleep_ms
from array import array

'''
class SingleSensor:
    def __init__(self, sensorPin):
        self.sensor = Pin(sensorPin, Pin.OUT_PP)
        self.defaultCentroid = 0

    def elapsed_time(self):
        self.sensor.init(Pin.OUT_PP)
        self.sensor.high()
        delay(1)
        self.sensor.init(Pin.IN)
        start_time = micros()
        while self.sensor.value() == 1:
            if micros() - start_time > 10000:  # Timeout to avoid infinite loop
                break
        elapsed_time = micros() - start_time

        return elapsed_time
'''

class LineSensor:

    def __init__(self, sensorPins):
        self.sensorList = [Pin(pin, Pin.OUT_PP) for pin in sensorPins]
        '''
        self.sensor_1 = Pin(pin_1, Pin.OUT_PP)
        self.sensor_2 = Pin(pin_2, Pin.OUT_PP)
        self.sensor_3 = Pin(pin_3, Pin.OUT_PP)
        self.sensor_4 = Pin(pin_4, Pin.OUT_PP)
        self.sensor_5 = Pin(pin_5, Pin.OUT_PP)
        self.sensor_6 = Pin(pin_6, Pin.OUT_PP)
        self.sensor_7 = Pin(pin_7, Pin.OUT_PP)
        self.sensor_8 = Pin(pin_8, Pin.OUT_PP)
        self.sensorList = [self.sensor_1,
                           self.sensor_2,
                           self.sensor_3,
                           self.sensor_4,
                           self.sensor_5,
                           self.sensor_6,
                           self.sensor_7,
                           self.sensor_7]
        '''

    def elapsed_times(self):
        elapsed_times = []
        for sensor in self.sensorList:
            sensor.init(Pin.OUT_PP)
            sensor.high()
            delay(1)
            sensor.init(Pin.IN)
            start_time = micros()
            while sensor.value() == 1:
                if micros() - start_time > 10000:  # Timeout to avoid infinite loop
                    break
            elapsed_time = micros() - start_time
            elapsed_times.append(elapsed_time)
        return elapsed_times
        '''
        for sensor in self.sensorList:
            sensor.init(Pin.OUT_PP)
            sensor.high()
        delay(1)
        for sensor in self.sensorList:
            sensor.init(Pin.IN)

        start_time = micros()
        elapsed_times = [None] * len(self.sensorList)

        print(self.sensorList)
        while any(sensor.value() == 1 for sensor in self.sensorList):
            current_time = micros()
            for i, sensor in enumerate(self.sensorList):
                if sensor.value() == 1 and elapsed_times[i] is None:
                    elapsed_times[i] = current_time - start_time

                if current_time - start_time > 10000:  # Timeout to prevent infinite loop
                    break

        return(elapsed_times)
        '''

    def read(self):
        sensorTimes = self.elapsed_times()
       
        m_sum = 0
        xm_sum = 0
        for i in range(len(sensorTimes)):
            x = i +.5
            m = sensorTimes[i]
            xm_sum += x*m
            m_sum += m
        centroid = 4-(xm_sum/m_sum)-.04
        thickness = 0
        for time in sensorTimes:
            if time > 2000:
                thickness += 1
            if time > 1500:
                thickness +=.5
        
        returnArray = [round(centroid,4),thickness]
        return returnArray

    
            
        
        
        '''
        # Test these thresholds and adjust through trial and error
        black_thresh_min = 0
        black_thresh_max = 1000
        gray_thresh_min = 1001
        gray_thresh_max = 1500
        # white_thresh_min = 1501
        # white_thresh_max = 2500
        for i in range(len(sensor_times)):
            if black_thresh_min <= sensor_times[i] <= black_thresh_max:
                sensor_times[i] = 0
            elif gray_thresh_min <= sensor_times[i] <= gray_thresh_max:
                sensor_times[i] = 0.5
            else:
                sensor_times[i] = 1

        return (sensor_times)
        '''

if __name__ == '__main__':
    """
    sensor_pin_7 = Pin.cpu.C2
    sensor_pin_6 = Pin.cpu.C3
    sensor_pin_5 = Pin.cpu.C4
    sensor_pin_4 = Pin.cpu.B1
    sensor_pin_3 = Pin.cpu.C5
    sensor_pin_2 = Pin.cpu.A7
    sensor_pin_1 = Pin.cpu.A6
    sensor_pin_0 = Pin.cpu.B13
    """

    # sensor1 = SingleSensor('B13')
    # sensor1.elapsed_time

    sensorPins = ['B13', 'A6', 'A7', 'C5', 'B1', 'C4', 'C3', 'C2']
    

    line_sensor = LineSensor(sensorPins)

    while (True):
        # print(sensor1.elapsed_time())
        # print(line_sensor.elapsed_time())
        
        print(line_sensor.read())
        sleep_ms(100)