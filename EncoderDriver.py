'''!@file
    @brief
    @details
    @author
    @date
'''

from pyb import Pin, Timer
from time import sleep_ms


class Encoder:
    '''!@brief
        @details
    '''
    def __init__(self, ENC_CH_A, ENC_CH_B, TIM):
        '''!@brief
            @details
            @param
            @param
            @param
        '''
        self.CHA = TIM.channel(1, pin=ENC_CH_A, mode=Timer.ENC_AB)
        self.CHB = TIM.channel(2, pin=ENC_CH_B, mode=Timer.ENC_AB)
        self.TIM = TIM
        self.position = 0
        self.count = 0
        self.delta = 0

    def update(self):
        '''!@brief
            @details
        '''
        newCount = self.TIM.counter()
        self.delta = newCount-self.count
        if self.delta > (65536/2):
            self.delta -= 65536
        if self.delta < (-65536/2):
            self.delta += 65536
        self.position += self.delta
        self.count = newCount
    
    def get_position(self):
        '''!@brief
            @details
            @return
        '''
        return self.position
        #return None
    
    def get_delta(self):
        '''!@brief
            @details
            @return
        '''
        return self.delta

    
    def zero(self):
        '''!@brief
            @details
        '''
        self.position = 0
     
 
