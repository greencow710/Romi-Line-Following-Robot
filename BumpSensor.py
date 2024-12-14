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

