<<<<<<< HEAD:Firmware/task_estimator.py
import micropython
from utime import ticks_us, ticks_diff
from ulab import numpy as np
import math

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)

class task_estimator:
    def __init__(self,
                 Ad, Bd,
                 uL_V_share, uR_V_share,
                 sL_m_share, sR_m_share,
                 psi_deg_share, psidot_dps_share,
                 xhat_shares,
                 yhat_shares=None,
                 posX_share=None, posY_share=None,
                 period_ms=20, lineLostCount=None,
                 reset_pos_go=None,
                 initX_share=None, initY_share=None):

        self._state = S0_INIT

        self.Ad = np.array(Ad, dtype=np.float)
        self.Bd = np.array(Bd, dtype=np.float)

        self._uL = uL_V_share
        self._uR = uR_V_share
        self._sL = sL_m_share
        self._sR = sR_m_share
        self._psi_deg = psi_deg_share
        self._psidot_dps = psidot_dps_share
        self._lineLostCount = lineLostCount

        self._xhat_out = xhat_shares
        self._yhat_out = yhat_shares
        self._posX = posX_share
        self._posY = posY_share

        self._period_ms = period_ms
        self._tprev = 0

        self._reset_pos_go = reset_pos_go
        self._initX = initX_share
        self._initY = initY_share

        self._xhat = np.zeros((4, 1), dtype=np.float)

        self._psi_prev = None
        self._psi_unwrapped = 0.0

        self._sL_prev = 0.0
        self._sR_prev = 0.0

        self._posx = 0.100
        self._posy = 0.800

    def _unwrap_deg_to_rad(self, psi_deg):
        psi_rad = psi_deg * (math.pi / 180.0)

        if self._psi_prev is None:
            self._psi_prev = psi_rad
            self._psi_unwrapped = psi_rad
            return self._psi_unwrapped

        d = psi_rad - self._psi_prev

        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi

        self._psi_unwrapped += d
        self._psi_prev = psi_rad
        return self._psi_unwrapped

    def _write_outputs(self):
        for i in range(4):
            try:
                self._xhat_out[i].put(float(self._xhat[i, 0]))
            except Exception:
                pass

        if self._yhat_out is not None:
            for i in range(4):
                try:
                    self._yhat_out[i].put(float(self._xhat[i, 0]))
                except Exception:
                    pass

        if self._posX is not None:
            self._posX.put(self._posx)

        if self._posY is not None:
            self._posY.put(self._posy)

    def _reset_position(self):
        if self._initX is not None:
            self._posx = float(self._initX.get())
        else:
            self._posx = 0.100

        if self._initY is not None:
            self._posy = float(self._initY.get())
        else:
            self._posy = 0.800

        self._sL_prev = float(self._sL.get())
        self._sR_prev = float(self._sR.get())

        self._psi_prev = None
        self._psi_unwrapped = 0.0

        self._write_outputs()

    def run(self):
        while True:
            if self._state == S0_INIT:
                self._tprev = ticks_us()
                self._reset_position()
                self._state = S1_RUN

            elif self._state == S1_RUN:
                if self._reset_pos_go is not None and self._reset_pos_go.get():
                    self._reset_position()
                    self._reset_pos_go.put(False)
                    self._tprev = ticks_us()
                    yield self._state
                    continue

                tnow = ticks_us()
                dt_us = ticks_diff(tnow, self._tprev)
                self._tprev = tnow

                dt = dt_us / 1000000.0
                if dt <= 0.0 or dt > 0.5:
                    dt = self._period_ms / 1000.0

                uL = float(self._uL.get())
                uR = float(self._uR.get())
                sL = float(self._sL.get())
                sR = float(self._sR.get())
                psi = self._unwrap_deg_to_rad(float(self._psi_deg.get()))
                psidot = float(self._psidot_dps.get()) * (math.pi / 180.0)

                utilde = np.array([[uL],
                                   [uR],
                                   [sL],
                                   [sR],
                                   [psi],
                                   [psidot]], dtype=np.float)

                self._xhat = np.dot(self.Ad, self._xhat) + np.dot(self.Bd, utilde)

                dsL = sL - self._sL_prev
                dsR = sR - self._sR_prev

                self._sL_prev = sL
                self._sR_prev = sR

                ds = 0.5 * (dsL + dsR)

                if ds > 0.05:
                    ds = 0.05
                elif ds < -0.05:
                    ds = -0.05

                self._posx += ds * math.cos(psi)
                self._posy -= ds * math.sin(psi)

                self._write_outputs()

            yield self._state
=======
import micropython
from utime import ticks_us, ticks_diff
from ulab import numpy as np
import math

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)

class task_estimator:
    def __init__(self,
                 Ad, Bd,
                 uL_V_share, uR_V_share,
                 sL_m_share, sR_m_share,
                 psi_deg_share, psidot_dps_share,
                 xhat_shares,
                 yhat_shares=None,
                 posX_share=None, posY_share=None,
                 period_ms=20, lineLostCount=None,
                 reset_pos_go=None,
                 initX_share=None, initY_share=None):

        self._state = S0_INIT

        self.Ad = np.array(Ad, dtype=np.float)
        self.Bd = np.array(Bd, dtype=np.float)

        self._uL = uL_V_share
        self._uR = uR_V_share
        self._sL = sL_m_share
        self._sR = sR_m_share
        self._psi_deg = psi_deg_share
        self._psidot_dps = psidot_dps_share
        self._lineLostCount = lineLostCount

        self._xhat_out = xhat_shares
        self._yhat_out = yhat_shares
        self._posX = posX_share
        self._posY = posY_share

        self._period_ms = period_ms
        self._tprev = 0

        self._reset_pos_go = reset_pos_go
        self._initX = initX_share
        self._initY = initY_share

        self._xhat = np.zeros((4, 1), dtype=np.float)

        self._psi_prev = None
        self._psi_unwrapped = 0.0

        self._sL_prev = 0.0
        self._sR_prev = 0.0

        self._posx = 0.100
        self._posy = 0.800

    def _unwrap_deg_to_rad(self, psi_deg):
        psi_rad = psi_deg * (math.pi / 180.0)

        if self._psi_prev is None:
            self._psi_prev = psi_rad
            self._psi_unwrapped = psi_rad
            return self._psi_unwrapped

        d = psi_rad - self._psi_prev

        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi

        self._psi_unwrapped += d
        self._psi_prev = psi_rad
        return self._psi_unwrapped

    def _write_outputs(self):
        for i in range(4):
            try:
                self._xhat_out[i].put(float(self._xhat[i, 0]))
            except Exception:
                pass

        if self._yhat_out is not None:
            for i in range(4):
                try:
                    self._yhat_out[i].put(float(self._xhat[i, 0]))
                except Exception:
                    pass

        if self._posX is not None:
            self._posX.put(self._posx)

        if self._posY is not None:
            self._posY.put(self._posy)

    def _reset_position(self):
        if self._initX is not None:
            self._posx = float(self._initX.get())
        else:
            self._posx = 0.100

        if self._initY is not None:
            self._posy = float(self._initY.get())
        else:
            self._posy = 0.800

        self._sL_prev = float(self._sL.get())
        self._sR_prev = float(self._sR.get())

        self._psi_prev = None
        self._psi_unwrapped = 0.0

        self._write_outputs()

    def run(self):
        while True:
            if self._state == S0_INIT:
                self._tprev = ticks_us()
                self._reset_position()
                self._state = S1_RUN

            elif self._state == S1_RUN:
                if self._reset_pos_go is not None and self._reset_pos_go.get():
                    self._reset_position()
                    self._reset_pos_go.put(False)
                    self._tprev = ticks_us()
                    yield self._state
                    continue

                tnow = ticks_us()
                dt_us = ticks_diff(tnow, self._tprev)
                self._tprev = tnow

                dt = dt_us / 1000000.0
                if dt <= 0.0 or dt > 0.5:
                    dt = self._period_ms / 1000.0

                uL = float(self._uL.get())
                uR = float(self._uR.get())
                sL = float(self._sL.get())
                sR = float(self._sR.get())
                psi = self._unwrap_deg_to_rad(float(self._psi_deg.get()))
                psidot = float(self._psidot_dps.get()) * (math.pi / 180.0)

                utilde = np.array([[uL],
                                   [uR],
                                   [sL],
                                   [sR],
                                   [psi],
                                   [psidot]], dtype=np.float)

                self._xhat = np.dot(self.Ad, self._xhat) + np.dot(self.Bd, utilde)

                dsL = sL - self._sL_prev
                dsR = sR - self._sR_prev

                self._sL_prev = sL
                self._sR_prev = sR

                ds = 0.5 * (dsL + dsR)

                if ds > 0.05:
                    ds = 0.05
                elif ds < -0.05:
                    ds = -0.05

                self._posx += ds * math.cos(psi)
                self._posy -= ds * math.sin(psi)

                self._write_outputs()

            yield self._state
>>>>>>> ee3ec90 (update files, docs, reports):firmware/task_estimator.py
