from time import ticks_us, ticks_diff   
from array import array
from pyb import Pin, Timer
class encoder:


    def __init__(self, tim, chA_pin, chB_pin):
        '''Initializes an Encoder object'''
        self.tim= Timer(tim, period=0xFFFF, prescaler = 0)
        self.tim.channel(1, pin=chA_pin, mode=Timer.ENC_AB)
        self.tim.channel(2, pin=chB_pin, mode=Timer.ENC_AB)
		
        self.position   = 0    
        self.prev_position = 0  
        self.delta      = 0    
        self.dt         = 0    
        self.earlier    = 0
        self.now        = 0
    def update(self):
        self.position = -self.tim.counter()
        self.delta = self.position - self.prev_position
        self.prev_position = self.position

        if self.delta > 32768:
            self.delta -= 65536
        elif self.delta < -32768:
            self.delta += 65536

        self.now = ticks_us()
        self.dt = ticks_diff(self.now, self.earlier)
        self.earlier = self.now

    def get_position(self):
        
        return self.position
            
    def get_velocity(self):
        
        return self.delta/self.dt
    
    def zero(self):
        self.prev_position = self.tim.counter()
        self.position = 0
pass