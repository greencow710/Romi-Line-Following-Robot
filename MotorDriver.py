# -*- coding: utf-8 -*-
"""
Created on Thu Oct  3 12:20:32 2024

@author: green
"""

from pyb import Pin, Timer

class Motor:

    def __init__(self, PWM_tim, ENA_pin, DIR_pin, PWM_pin):
        self.ENA = Pin(ENA_pin, mode=Pin.OUT_PP)  # ENA
        self.DIR = Pin(DIR_pin, mode=Pin.OUT_PP)  # ENA
        # IN1 = Pin(Pin.cpu.IN1_pin, mode=Pin.OUT_PP) # IN1
        # IN2 = Pin(Pin.cpu.IN2_pin, mode=Pin.OUT_PP) # IN2

        self.CH1 = PWM_tim.channel(1, pin=PWM_pin, mode=Timer.PWM)
        #self.CH2 = PWM_tim.channel(2, pin=IN2_pin, mode=Timer.PWM)

        # pass

    def set_duty(self, duty):
        if duty < 0:
            duty = duty*(-1)
            self.CH1.pulse_width_percent(duty)
            self.DIR.low()
        else:
            self.CH1.pulse_width_percent(duty)
            self.DIR.high()
        # pass

    def enable(self):
        self.ENA.high()
        # pass

    def disable(self):
        self.ENA.low()
        
