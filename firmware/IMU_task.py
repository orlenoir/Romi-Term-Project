# -*- coding: utf-8 -*-
"""
Created on Tue Feb 24 08:52:39 2026

@author: orlen
"""

# task_imu.py
import micropython

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)

class task_imu:
    def __init__(self, imu, heading_share, yawrate_share, calraw_share=None):
        self._state = S0_INIT
        self._imu = imu
        self._heading = heading_share
        self._yawrate = yawrate_share
        self._calraw = calraw_share

    def run(self):
        while True:
            if self._state == S0_INIT:
                self._state = S1_RUN

            elif self._state == S1_RUN:
                h = self._imu.heading_deg()
                r = self._imu.yaw_rate_dps()
                self._heading.put(float(h))
                self._yawrate.put(float(r))

                if self._calraw is not None:
                    self._calraw.put(self._imu.get_cal_status()["raw"])

            yield self._state