from motor_driver import motor_driver
from encoder      import encoder
from task_share   import Share, Queue
from utime        import ticks_us, ticks_diff
import micropython
import math

#inital closed loop control values

v_out=0


S0_INIT = micropython.const(0) # State 0 - initialiation
S1_WAIT = micropython.const(1) # State 1 - wait for go command
S2_RUN  = micropython.const(2) # State 2 - run closed loop control

class task_motor:


    def __init__(self,
                 mot: motor_driver, enc: encoder,
                 goFlag: Share, dataValues: Queue, timeValues: Queue,kp,ki,vi,
                 uV_share=None, s_m_share=None,
                 Vbatt=6.0, wheel_r=0.035, cpr=1440):


        self._state: int        = S0_INIT    # The present state of the task       
        
        self._mot: motor_driver = mot        # A motor object
        
        self._enc: encoder      = enc        # An encoder object
        
        self._goFlag: Share     = goFlag     # A share object representing a
                                             # flag to start data collection
        
        self._dataValues: Queue = dataValues # A queue object used to store
                                             # collected encoder position
        
        self._timeValues: Queue = timeValues # A queue object used to store the
                                             # time stamps associated with the
                                             # collected encoder data
        
        self._startTime: int    = 0          # The start time (in microseconds)
                                             # for a batch of collected data
        self._kp = kp
        self._ki = ki
        self._vi = vi
        self.integral=0.0
        
        self._uV = uV_share
        self._s_m = s_m_share
        self._Vbatt = Vbatt
        self._wheel_r = wheel_r
        self._cpr = cpr

        print("Motor Task object instantiated")
        
    def run(self):
        
        global effort
        effort=0
            
        while True:
            
            if self._state == S0_INIT: 
                self._state = S1_WAIT
                
            elif self._state == S1_WAIT: 
                if self._goFlag.get():
                 
                    self._startTime = ticks_us()
                    self._state = S2_RUN
                    self.integral = 0.0
                    self._enc.update()

                
            elif self._state == S2_RUN:
                kp = self._kp.get()
                ki = self._ki.get()
                vi = self._vi.get()
                self._enc.update()
                pos=self._enc.get_position()

                w_enc=self._enc.get_velocity()
                v_out=w_enc*2*math.pi*0.035*10**6/1440  

                
                t=ticks_us()

                if (not self._dataValues.full()) and (not self._timeValues.full()):
                    self._dataValues.put(v_out)
                    self._timeValues.put(ticks_diff(t, self._startTime))

                if vi>0.55:
                    vi=0.55
                elif vi<-0.55:
                    vi=-0.55

                r=vi
                e=(r-v_out)*100/.55
                self.integral+= e*self._enc.dt/1000000
                if effort >100:
                    effort=kp*e
                elif effort <-100:
                    effort=kp*e
                else:
                    effort=kp*e+ki*self.integral
                
                    

                self._mot.set_effort(effort)

                if self._uV is not None:
                    u_volts = (effort / 100.0) * self._Vbatt
                    self._uV.put(u_volts)
                
                if self._s_m is not None:
                    counts = self._enc.get_position()
                    s_m = (counts / self._cpr) * (2.0 * math.pi * self._wheel_r)
                    self._s_m.put(s_m)
                time=[]
                velocity=[]
                time.append(ticks_diff(t,self._startTime))
                velocity.append(v_out)

                

            yield self._state

          
