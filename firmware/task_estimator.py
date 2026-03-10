# -*- coding: utf-8 -*-
"""
Created on Tue Feb 24 10:16:26 2026

@author: orlen
"""

# task_estimator.py
import micropython
from utime import ticks_us, ticks_diff
from ulab import numpy as np
import math

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)

class task_estimator:
    """
    Discrete observer update:
        xhat[k+1] = Ad * xhat[k] + Bd * u_tilde[k]
    where u_tilde = [u_L, u_R, y1, y2, y3, y4]^T
    and y = [s_L, s_R, psi, psi_dot]^T

    Units:
      - s_L, s_R in meters
      - psi in radians (unwrap handled)
      - psi_dot in rad/s
      - u_L, u_R in volts (applied motor voltage estimate)
    """

    def __init__(self,
                 Ad, Bd,
                 uL_V_share, uR_V_share,
                 sL_m_share, sR_m_share,
                 psi_deg_share, psidot_dps_share,
                 # outputs
                 xhat_shares,          # list/tuple of 4 Share('f')
                 yhat_shares=None,     # optional list/tuple of 4 Share('f')
                 posX_share=None, posY_share=None,   # optional predicted position (m)
                 period_ms=20):

        self._state = S0_INIT

        # Store matrices as ulab arrays
        self.Ad = np.array(Ad, dtype=np.float)
        self.Bd = np.array(Bd, dtype=np.float)

        # Inputs
        self._uL = uL_V_share
        self._uR = uR_V_share
        self._sL = sL_m_share
        self._sR = sR_m_share
        self._psi_deg = psi_deg_share
        self._psidot_dps = psidot_dps_share

        # Outputs
        self._xhat_out = xhat_shares
        self._yhat_out = yhat_shares
        self._posX = posX_share
        self._posY = posY_share

        # Timing
        self._period_ms = period_ms
        self._tprev = 0

        # State memory
        self._xhat = np.zeros((4, 1), dtype=np.float)

        # For heading unwrap + position integration
        self._psi_prev = None
        self._psi_unwrapped = 0.0

        self._sL_prev = None
        self._sR_prev = None
        self._posx = 0.0
        self._posy = 0.0

    def _unwrap_deg_to_rad(self, psi_deg):
        """Convert deg to rad and unwrap to keep continuity."""
        psi_rad = psi_deg * (math.pi / 180.0)
        if self._psi_prev is None:
            self._psi_prev = psi_rad
            self._psi_unwrapped = psi_rad
            return self._psi_unwrapped

        d = psi_rad - self._psi_prev
        # unwrap around +/-pi
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi

        self._psi_unwrapped += d
        self._psi_prev = psi_rad
        return self._psi_unwrapped

    def _write_outputs(self):
        # xhat
        for i in range(4):
            try:
                self._xhat_out[i].put(float(self._xhat[i, 0]))
            except Exception:
                pass

        # yhat: if your C is identity (common for this reduced model), yhat == xhat
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

    def run(self):
        while True:
            if self._state == S0_INIT:
                self._tprev = ticks_us()
                self._state = S1_RUN

            elif self._state == S1_RUN:
                tnow = ticks_us()
                dt_us = ticks_diff(tnow, self._tprev)
                self._tprev = tnow

                # Use fixed dt from task period if dt_us is weird
                dt = dt_us / 1_000_000.0
                if dt <= 0.0 or dt > 0.5:
                    dt = self._period_ms / 1000.0

                # --- Read inputs ---
                uL = float(self._uL.get())
                uR = float(self._uR.get())

                sL = float(self._sL.get())          # meters
                sR = float(self._sR.get())          # meters

                psi = self._unwrap_deg_to_rad(float(self._psi_deg.get()))
                psidot = float(self._psidot_dps.get()) * (math.pi / 180.0)  # deg/s -> rad/s

                # u_tilde = [uL, uR, sL, sR, psi, psidot]^T
                utilde = np.array([[uL],
                                   [uR],
                                   [sL],
                                   [sR],
                                   [psi],
                                   [psidot]], dtype=np.float)

                # --- Observer update ---
                # xhat[k+1] = Ad*xhat[k] + Bd*utilde[k]
                self._xhat = np.dot(self.Ad, self._xhat) + np.dot(self.Bd, utilde)

                # --- Optional: predict global position using estimated motion ---
                # Treat xhat[0], xhat[1] as sL, sR estimates (m)
                sL_hat = float(self._xhat[0, 0])
                sR_hat = float(self._xhat[1, 0])
                psi_hat = float(self._xhat[2, 0])

                if self._sL_prev is None:
                    self._sL_prev = sL_hat
                    self._sR_prev = sR_hat
                else:
                    vL = (sL_hat - self._sL_prev) / dt
                    vR = (sR_hat - self._sR_prev) / dt
                    self._sL_prev = sL_hat
                    self._sR_prev = sR_hat

                    v = 0.5 * (vL + vR)
                    self._posx += v * math.cos(psi_hat) * dt
                    self._posy += v * math.sin(psi_hat) * dt

                # Publish
                self._write_outputs()

            yield self._state