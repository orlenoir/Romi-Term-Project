import micropython
from utime import ticks_us, ticks_diff

S0_INIT = micropython.const(0)
S1_RUN = micropython.const(1)

class task_line:
    def __init__(self, sensor, vi_L, vi_R, lineGo=None, kpline=None, kiline=None, base_vi=None,
                 timeQ=None, centroidQ=None, errorQ=None, deltaQ=None, integralQ=None,
                 lineLostCount=None, recoverGo=None,
                 imu_heading_abs_share=None, imu_heading0_share=None,
                 blackStopGo=None, blackDetectEnable=None, black_threshold=2500,
                 posX_share=None, posY_share=None,
                 final_line_exit_y_m=0.350, return_line_lost_threshold=400,
                 initial_fast_x_m=1.300, initial_fast_speed=0.10,
                 first_line_lost_threshold=450):

        self._state = S0_INIT
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
        self._integralQ = integralQ
        self._lineLostCount = lineLostCount
        self._recoverGo = recoverGo
        self._imu_heading_abs = imu_heading_abs_share
        self._imu_heading0 = imu_heading0_share
        self._blackStopGo = blackStopGo
        self._blackDetectEnable = blackDetectEnable
        self._black_threshold = int(black_threshold)

        self._posX = posX_share
        self._posY = posY_share
        self._final_line_exit_y = float(final_line_exit_y_m)
        self._return_line_lost_threshold = int(return_line_lost_threshold)
        self._initial_fast_x = float(initial_fast_x_m)
        self._initial_fast_speed = float(initial_fast_speed)
        self._first_line_lost_threshold = int(first_line_lost_threshold)

        self._t0 = 0
        self._tprev = 0
        self._integral = 0.0
        self._last_e = 0.0
        self._lostAlready = False

    def _clamp(self, x, lo, hi):
        if x < lo:
            return lo
        if x > hi:
            return hi
        return x

    def _avg_raw(self):
        vals = self._sensor.read_raw()
        return sum(vals) / len(vals)

    def _final_line_loss_allowed(self, avg_raw):
        if self._lineLostCount is None:
            return False

        if int(self._lineLostCount.get()) != 1:
            return False

        if self._blackDetectEnable is None:
            return False

        if self._blackDetectEnable.get():
            return False

        if self._posY is None:
            return False

        y_ok = float(self._posY.get()) > self._final_line_exit_y
        line_gone = avg_raw < self._return_line_lost_threshold
        return y_ok and line_gone

    def _normal_line_loss_allowed(self, avg_raw):
        if self._lineLostCount is None:
            return avg_raw < self._first_line_lost_threshold

        return int(self._lineLostCount.get()) == 0 and avg_raw < self._first_line_lost_threshold

    def _active_base_speed(self):
        base = self._base_vi.get() if self._base_vi is not None else 0.0

        if self._lineLostCount is None or self._posX is None:
            return base

        if int(self._lineLostCount.get()) == 0 and float(self._posX.get()) < self._initial_fast_x:
            return self._initial_fast_speed

        return base

    def run(self):
        while True:
            if self._state == S0_INIT:
                self._t0 = ticks_us()
                self._tprev = self._t0
                self._integral = 0.0
                self._last_e = 0.0
                self._lostAlready = False
                self._state = S1_RUN

            elif self._state == S1_RUN:
                if (self._lineGo is not None) and (not self._lineGo.get()):
                    yield self._state
                    continue

                avg_raw = self._avg_raw()

                if (self._blackDetectEnable is not None) and self._blackDetectEnable.get():
                    if avg_raw > self._black_threshold:
                        self._vi_L.put(0.0)
                        self._vi_R.put(0.0)

                        if self._lineGo is not None:
                            self._lineGo.put(False)

                        if self._blackStopGo is not None:
                            self._blackStopGo.put(True)

                        if self._blackDetectEnable is not None:
                            self._blackDetectEnable.put(False)

                        yield self._state
                        continue

                final_loss = self._final_line_loss_allowed(avg_raw)
                normal_loss = self._normal_line_loss_allowed(avg_raw)

                if final_loss or normal_loss:
                    if not self._lostAlready:
                        self._lostAlready = True

                        if self._lineLostCount is not None:
                            self._lineLostCount.put(self._lineLostCount.get() + 1)

                        if self._recoverGo is not None:
                            self._recoverGo.put(True)

                        if self._lineGo is not None:
                            self._lineGo.put(False)

                    yield self._state
                    continue
                else:
                    self._lostAlready = False

                c = self._sensor.centroid()
                if c is None:
                    yield self._state
                    continue

                e = c - 3.5

                t = ticks_us()
                dt_us = ticks_diff(t, self._tprev)
                self._tprev = t
                dt = dt_us / 1000000.0 if dt_us > 0 else 0.0

                e_use = float(e)
                self._last_e = e_use

                kp = self._kpline.get() if self._kpline is not None else 0.0
                ki = self._kiline.get() if self._kiline is not None else 0.0

                self._integral += e_use * dt

                alpha = 1.5
                Imax = (alpha * kp / ki) * abs(e_use) if ki != 0 else 0.0

                if self._integral > Imax:
                    self._integral = Imax
                elif self._integral < -Imax:
                    self._integral = -Imax

                dv = (kp * e_use + ki * self._integral) / 25.0
                base = self._active_base_speed()
                dv = self._clamp(dv, -abs(2*base), abs(2*base))

                vL = base - dv
                vR = base + dv
                self._vi_L.put(vL)
                self._vi_R.put(vR)

                if self._timeQ is not None and (not self._timeQ.full()):
                    self._timeQ.put(ticks_diff(t, self._t0))
                if self._centroidQ is not None and (not self._centroidQ.full()):
                    self._centroidQ.put(float(c))
                if self._errorQ is not None and (not self._errorQ.full()):
                    self._errorQ.put(float(e))
                if self._deltaQ is not None and (not self._deltaQ.full()):
                    self._deltaQ.put(float(dv))
                if self._integralQ is not None and (not self._integralQ.full()):
                    self._integralQ.put(float(self._integral))

            yield self._state