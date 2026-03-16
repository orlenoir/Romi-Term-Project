
from pyb import Pin, Timer


class motor_driver:    
    def __init__(self, PWM, DIR, nSLP):
        self.pwm=PWM
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)   
        self.DIR_pin=Pin(DIR, mode=Pin.OUT_PP, value=0) 
        self.nSLP_pin.low()
        self.pwm.pulse_width_percent(0)

    def set_effort(self, effort):
        if effort < -100:
            effort = -100
        elif effort > 100:
            effort = 100
        
        if effort < 0:
            self.DIR_pin.high()
            effort=-effort
        elif effort >= 0: 
            self.DIR_pin.low()
            
        self.pwm.pulse_width_percent(int(effort))

    def enable(self):
        self.set_effort(0)
        self.nSLP_pin.high()

    def disable(self):
        self.nSLP_pin.low()
        self.set_effort(0)
