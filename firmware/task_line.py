# task_line.py
import micropython
from utime import ticks_us, ticks_diff

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)


class task_line:
    def __init__(self,sensor,vi_L, vi_R, lineGo=None, kpline=None, kiline=None, base_vi=None, 
                 timeQ=None, centroidQ=None, errorQ=None, deltaQ=None, integralQ=None):

        self._state = S0_INIT
        self._integralQ = integralQ
        self._sensor = sensor
        self._vi_L = vi_L
        self._vi_R = vi_R

        self._lineGo = lineGo
        self._kpline = kpline
        self._kiline = kiline

        self._base_vi = base_vi

        self._timeQ = timeQ
        self._centroidQ = centroidQ
        self._errorQ = errorQ
        self._deltaQ = deltaQ

        self._t0 = 0
        self._tprev = 0
        self._integral = 0.0
        self._last_e = 0.0

    def _clamp(self, x, lo, hi):
        if x < lo:
            return lo
        if x > hi:
            return hi
        return x

    def run(self):
        while True:
            if self._state == S0_INIT:
                self._t0 = ticks_us()
                self._tprev = self._t0
                self._integral = 0.0
                self._last_e = 0.0
                self._state = S1_RUN

            elif self._state == S1_RUN:

                if (self._lineGo is not None) and (not self._lineGo.get()):
                    yield self._state
                    continue

                c = self._sensor.centroid()
                e = None if c is None else (c - 3.5)

                t = ticks_us()
                dt_us = ticks_diff(t, self._tprev)
                self._tprev = t
                dt = dt_us / 1_000_000.0 if dt_us > 0 else 0.0

                if e is None or c is None:
                    e_use = self._last_e
                else:
                    e_use = float(e)
                    self._last_e = e_use

                kp = self._kpline.get() if self._kpline is not None else 0.00
                ki = self._kiline.get() if self._kiline is not None else 0.00

                self._integral += e_use * dt

                alpha = 3.0
                Imax = (alpha * kp / ki) * abs(e_use) if ki != 0 else 0.0

                if self._integral > Imax:
                    self._integral = Imax
                elif self._integral < -Imax:
                    self._integral = -Imax
                


                dv = (kp * e_use + ki * self._integral)/25
                try:
                    base = self._base_vi.get()
                except AttributeError:
                    base = self._base_v
                dv = self._clamp(dv, -abs(base), abs(base))

                vL = base - dv
                vR = base + dv
                self._vi_L.put(vL)
                self._vi_R.put(vR)

                if self._timeQ is not None and (not self._timeQ.full()):
                    self._timeQ.put(ticks_diff(t, self._t0))
                if self._centroidQ is not None and (not self._centroidQ.full()):
                    self._centroidQ.put(float(c) if c is not None else -1.0)
                if self._errorQ is not None and (not self._errorQ.full()):
                    self._errorQ.put(float(e) if e is not None else 0.0)
                if self._deltaQ is not None and (not self._deltaQ.full()):
                    self._deltaQ.put(float(dv))
                if self._integralQ is not None and (not self._integralQ.full()):
                    self._integralQ.put(float(self._integral))

            yield self._state