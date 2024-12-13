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


if __name__ == '__main__':
    
   
    
   tim_2 = Timer(2, period = 65535, prescaler = 0)
   tim_3 = Timer(3, period = 65535, prescaler = 0)
   ENC_Mot_A = Encoder(Pin.cpu.A0, Pin.cpu.A1, tim_2)
   ENC_Mot_B = Encoder(Pin.cpu.B4, Pin.cpu.B5, tim_3)
    

   while True:
        ENC_Mot_A.update()
        ENC_Mot_B.update()
        print("\nMotor A:\nPosition: ", ENC_Mot_A.get_position())
        print("delta:", ENC_Mot_A.get_delta())
        print("Motor B:\nPosition: ", ENC_Mot_B.get_position())
        print("delta:", ENC_Mot_B.get_delta())
        sleep_ms(1000)
     
 
