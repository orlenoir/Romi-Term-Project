from motor_driver import motor_driver
from encoder import encoder
from task_share import Share, Queue
from utime import ticks_us, ticks_diff
import micropython
import math

v_out = 0

S0_INIT = micropython.const(0)
S1_WAIT = micropython.const(1)
S2_RUN  = micropython.const(2)

class task_motor:
    def __init__(self,
                 mot: motor_driver, enc: encoder,
                 goFlag: Share, dataValues: Queue, timeValues: Queue, kp, ki, vi,
                 uV_share=None, s_m_share=None,
                 Vbatt=6.0, wheel_r=0.035, cpr=1440, lineLostCount=None):

        self._state = S0_INIT
        self._mot = mot
        self._enc = enc
        self._goFlag = goFlag
        self._dataValues = dataValues
        self._timeValues = timeValues
        self._lineLostCount = lineLostCount
        self._startTime = 0
        self._kp = kp
        self._ki = ki
        self._vi = vi
        self.integral = 0.0

        self._uV = uV_share
        self._s_m = s_m_share
        self._Vbatt = Vbatt
        self._wheel_r = wheel_r
        self._cpr = cpr

        print("Motor Task object instantiated")

    def run(self):
        global effort
        effort = 0

        while True:
            if self._state == S0_INIT:
                self._mot.enable()
                self._mot.set_effort(0)
                self._enc.zero()
                self._startTime = ticks_us()
                self.integral = 0.0
                self._state = S1_WAIT

            elif self._state == S1_WAIT:
                self._mot.enable()
                self._mot.set_effort(0)

                if self._uV is not None:
                    self._uV.put(0.0)

                if self._s_m is not None:
                    counts = self._enc.get_position()
                    s_m = (counts / self._cpr) * (2.0 * math.pi * self._wheel_r)
                    self._s_m.put(s_m)

                if self._goFlag.get():
                    self._mot.enable()
                    self._startTime = ticks_us()
                    self.integral = 0.0
                    effort = 0
                    self._enc.update()
                    self._state = S2_RUN

            elif self._state == S2_RUN:
                if not self._goFlag.get():
                    self._mot.set_effort(0)

                    if self._uV is not None:
                        self._uV.put(0.0)

                    if self._s_m is not None:
                        counts = self._enc.get_position()
                        s_m = (counts / self._cpr) * (2.0 * math.pi * self._wheel_r)
                        self._s_m.put(s_m)

                    self.integral = 0.0
                    effort = 0
                    self._state = S1_WAIT
                    yield self._state
                    continue

                kp = self._kp.get()
                ki = self._ki.get()
                vi = self._vi.get()

                self._mot.enable()
                self._enc.update()

                if self._enc.dt <= 0:
                    w_enc = 0.0
                else:
                    w_enc = self._enc.get_velocity()

                v_out = w_enc * 2 * math.pi * self._wheel_r * 10**6 / self._cpr

                t = ticks_us()

                if self._dataValues is not None and self._timeValues is not None:
                    if (not self._dataValues.full()) and (not self._timeValues.full()):
                        self._dataValues.put(v_out)
                        self._timeValues.put(ticks_diff(t, self._startTime))

                if vi > 0.55:
                    vi = 0.55
                elif vi < -0.55:
                    vi = -0.55

                r = vi
                e = (r - v_out) * 100 / 0.55

                if self._enc.dt > 0:
                    self.integral += e * self._enc.dt / 1000000.0

                if effort > 100:
                    effort = kp * e
                elif effort < -100:
                    effort = kp * e
                else:
                    effort = kp * e + ki * self.integral

                if effort > 100:
                    effort = 100
                elif effort < -100:
                    effort = -100

                self._mot.set_effort(effort)

                if self._uV is not None:
                    u_volts = (effort / 100.0) * self._Vbatt
                    self._uV.put(u_volts)

                if self._s_m is not None:
                    counts = self._enc.get_position()
                    s_m = (counts / self._cpr) * (2.0 * math.pi * self._wheel_r)
                    self._s_m.put(s_m)

            yield self._state