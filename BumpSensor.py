# -*- coding: utf-8 -*-
"""
Created on Thu Dec  5 12:39:21 2024

@author: green
"""

from pyb import Pin
from time import sleep_ms


class BumpSensor:

    def __init__(self, pin_bump):
        self.sensor = Pin(pin_bump, Pin.IN, Pin.PULL_DOWN)

    def is_triggered(self):
        if self.sensor.value() == 1:
            return True
        else:
            return False


if __name__ == '__main__':

    left_sensor_pin = Pin.board.PC0
    right_sensor_pin = Pin.board.PC1

    left_sensor = BumpSensor(left_sensor_pin)
    right_sensor = BumpSensor(right_sensor_pin)

    while (True):
        print(left_sensor.is_triggered())
        print(right_sensor.is_triggered())
        sleep_ms(1000)

