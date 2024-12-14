# -*- coding: utf-8 -*-
"""
Created on Thu Dec  5 13:20:46 2024

@author: green
"""

from pyb import Pin, Timer, micros, delay
from time import sleep_ms
from array import array

class LineSensor:

    def __init__(self, sensorPins):
        self.sensorList = [Pin(pin, Pin.OUT_PP) for pin in sensorPins]

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
