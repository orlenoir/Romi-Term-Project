import micropython

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)

class task_imu:
    def __init__(self, imu, heading_share, yawrate_share, calraw_share=None,
                 lineLostCount=None, heading_abs_share=None, heading0_share=None):
        self._state = S0_INIT
        self._imu = imu
        self._heading = heading_share
        self._yawrate = yawrate_share
        self._calraw = calraw_share
        self._lineLostCount = lineLostCount
        self._heading_abs = heading_abs_share
        self._heading0 = heading0_share

    def _wrap_deg(self, ang):
        while ang >= 180.0:
            ang -= 360.0
        while ang < -180.0:
            ang += 360.0
        return ang

    def run(self):
        while True:
            if self._state == S0_INIT:
                self._state = S1_RUN

            elif self._state == S1_RUN:
                h_abs = float(self._imu.heading_deg())
                r_dps = float(self._imu.yaw_rate_dps())

                if self._heading_abs is not None:
                    self._heading_abs.put(h_abs)

                h_rel = h_abs
                if self._heading0 is not None:
                    h0 = float(self._heading0.get())
                    h_rel = self._wrap_deg(h_abs - h0)

                self._heading.put(h_rel)
                self._yawrate.put(r_dps)

                if self._calraw is not None:
                    self._calraw.put(self._imu.get_cal_status()["raw"])

            yield self._state