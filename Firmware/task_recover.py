<<<<<<< HEAD:firmware/task_recover.py
import micropython
import math
from utime import ticks_ms, ticks_diff
from heading_nav import HeadingNav

S0_IDLE = micropython.const(0)
S1_CAPTURE = micropython.const(1)
S2_FORWARD = micropython.const(2)
S3_TURN = micropython.const(3)
S4_DRIVE = micropython.const(4)
S5_BUMP_CAPTURE = micropython.const(5)
S6_BACKUP = micropython.const(6)
S7_TURN_EAST = micropython.const(7)
S8_RESUME_LINE = micropython.const(8)
S9_FINAL_CAPTURE = micropython.const(9)
S10_FINAL_FORWARD = micropython.const(10)
S11_FINAL_TURN = micropython.const(11)
S12_FINAL_DRIVE_XY1 = micropython.const(12)
S13_BACKUP_BEFORE_XY2 = micropython.const(13)
S14_TURN_90 = micropython.const(14)
S15_FINAL_DRIVE_XY2 = micropython.const(15)
S16_FINAL_DRIVE_XY3 = micropython.const(16)
S17_RESUME_LINE_FINAL = micropython.const(17)
S18_RETURN_CAPTURE = micropython.const(18)
S19_RETURN_DRIVE_MID1 = micropython.const(19)
S20_RETURN_DRIVE_MID2 = micropython.const(20)
S21_RETURN_DRIVE_XY1 = micropython.const(21)
S22_RETURN_DRIVE_XY2 = micropython.const(22)
S23_FINAL_SPIN = micropython.const(23)
S24_FINAL_STOP = micropython.const(24)


class task_recover:
    def __init__(self, lineGo, recoverGo, lineLostCount,
                 imu_heading_rel, vi_L, vi_R, sL_m_share, sR_m_share,
                 target_heading_share=None, bump_pin=None,
                 blackStopGo=None, blackDetectEnable=None,
                 posX_share=None, posY_share=None,
                 kpline_share=None, kiline_share=None,
                 baseLineV_share=None,
                 est_reset_go=None, initX_share=None, initY_share=None,
                 black_reset_x_m=1.175, black_reset_y_m=0.225,
                 return_reset_x_m=0.400, return_reset_y_m=0.500,
                 fwd_distance_m=0.100, base_fwd=0.08, turn_speed=0.08,
                 kp_heading=0.01, heading_tol=2.0,
                 recover_heading_deg=140.0, drive_heading_deg=180.0,
                 backup_distance_m=0.005, east_heading_deg=110.0,
                 backup_speed=0.06, final_preturn_m=0.01,
                 final_turn_heading_deg=50.0,
                 final_target_x_m=1.550, final_target_y_m=0.150,
                 final_target2_backup_m=0.030,
                 final_target2_heading_deg=90.0,
                 final_target2_x_m=1.300, final_target2_y_m=0.075,
                 final_target3_x_m=1.150, final_target3_y_m=0.050,
                 final_target_tol_m=0.030,
                 final_line_kp=16.0, final_line_ki=0.2, final_line_speed=0.045,
                 return_mid1_x_m=0.900, return_mid1_y_m=0.375,
                 return_mid2_x_m=0.950, return_mid2_y_m=0.600,
                 return_target1_x_m=0.150, return_target1_y_m=0.550,
                 return_target2_x_m=0.050, return_target2_y_m=0.800):

        self._state = S0_IDLE

        self._lineGo = lineGo
        self._recoverGo = recoverGo
        self._lineLostCount = lineLostCount

        self._imu_h = imu_heading_rel
        self._viL = vi_L
        self._viR = vi_R
        self._sL = sL_m_share
        self._sR = sR_m_share

        self._target_heading = target_heading_share
        self._bump_pin = bump_pin
        self._blackStopGo = blackStopGo
        self._blackDetectEnable = blackDetectEnable

        self._posX = posX_share
        self._posY = posY_share

        self._kpline = kpline_share
        self._kiline = kiline_share
        self._baseLineV = baseLineV_share

        self._est_reset_go = est_reset_go
        self._initX = initX_share
        self._initY = initY_share
        self._black_reset_x = float(black_reset_x_m)
        self._black_reset_y = float(black_reset_y_m)
        self._return_reset_x = float(return_reset_x_m)
        self._return_reset_y = float(return_reset_y_m)

        self._nav = HeadingNav(
            turn_speed=turn_speed,
            drive_speed=base_fwd,
            kp_drive=kp_heading,
            heading_tol=heading_tol
        )

        self._fwd_distance = float(fwd_distance_m)
        self._recover_heading = float(recover_heading_deg)
        self._drive_heading = float(drive_heading_deg)
        self._backup_distance = float(backup_distance_m)
        self._east_heading = float(east_heading_deg)
        self._backup_speed = float(backup_speed)

        self._final_preturn = float(final_preturn_m)
        self._final_turn_heading = float(final_turn_heading_deg)
        self._final_target_x = float(final_target_x_m)
        self._final_target_y = float(final_target_y_m)
        self._final_target2_backup = float(final_target2_backup_m)
        self._final_target2_heading = float(final_target2_heading_deg)
        self._final_target2_x = float(final_target2_x_m)
        self._final_target2_y = float(final_target2_y_m)
        self._final_target3_x = float(final_target3_x_m)
        self._final_target3_y = float(final_target3_y_m)
        self._final_target_tol = float(final_target_tol_m)

        self._final_line_kp = float(final_line_kp)
        self._final_line_ki = float(final_line_ki)
        self._final_line_speed = float(final_line_speed)

        self._return_mid1_x = float(return_mid1_x_m)
        self._return_mid1_y = float(return_mid1_y_m)
        self._return_mid2_x = float(return_mid2_x_m)
        self._return_mid2_y = float(return_mid2_y_m)
        self._return_target1_x = float(return_target1_x_m)
        self._return_target1_y = float(return_target1_y_m)
        self._return_target2_x = float(return_target2_x_m)
        self._return_target2_y = float(return_target2_y_m)

        self._start_dist = 0.0
        self._final_spin_speed = 0.4
        self._final_spin_time_ms = 700
        self._spin_start_ms = 0

    def _avg_dist(self):
        return 0.5 * (float(self._sL.get()) + float(self._sR.get()))

    def _set_target(self, ang):
        self._nav.set_goal_heading(ang)
        if self._target_heading is not None:
            self._target_heading.put(self._nav.goal_angle)

    def _turn_step(self):
        h = float(self._imu_h.get())
        vL, vR, done = self._nav.turn_to_heading(h)
        self._viL.put(vL)
        self._viR.put(vR)
        return done

    def _drive_heading_step(self):
        h = float(self._imu_h.get())
        vL, vR = self._nav.go_straight(h)
        self._viL.put(vL)
        self._viR.put(vR)

    def _drive_to_xy_step(self, xg, yg):
        x = 0.0 if self._posX is None else float(self._posX.get())
        y = 0.0 if self._posY is None else float(self._posY.get())
        dx = xg - x
        dy = yg - y
        dist = math.sqrt(dx * dx + dy * dy)
        self._set_target(math.degrees(math.atan2(-dy, dx)))
        self._drive_heading_step()
        return dist

    def _reset_xy(self, x, y):
        if self._initX is not None:
            self._initX.put(x)
        if self._initY is not None:
            self._initY.put(y)
        if self._posX is not None:
            self._posX.put(x)
        if self._posY is not None:
            self._posY.put(y)
        if self._est_reset_go is not None:
            self._est_reset_go.put(True)

    def run(self):
        while True:
            if self._state == S0_IDLE:
                if self._blackStopGo is not None and self._blackStopGo.get():
                    self._state = S9_FINAL_CAPTURE
                    yield self._state
                    continue

                if self._recoverGo.get() and (not self._lineGo.get()):
                    if self._lineLostCount is not None and int(self._lineLostCount.get()) >= 2:
                        self._state = S18_RETURN_CAPTURE
                    else:
                        self._state = S1_CAPTURE

                yield self._state
                continue

            if self._state == S1_CAPTURE:
                self._set_target(float(self._imu_h.get()))
                self._start_dist = self._avg_dist()
                self._state = S2_FORWARD
                yield self._state
                continue

            if self._state == S2_FORWARD:
                self._drive_heading_step()
                if abs(self._avg_dist() - self._start_dist) >= self._fwd_distance:
                    self._set_target(self._recover_heading)
                    self._state = S3_TURN
                yield self._state
                continue

            if self._state == S3_TURN:
                if self._turn_step():
                    self._set_target(self._drive_heading)
                    self._recoverGo.put(False)
                    self._state = S4_DRIVE
                yield self._state
                continue

            if self._state == S4_DRIVE:
                if self._bump_pin is not None and self._bump_pin.value() == 0:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S5_BUMP_CAPTURE
                    yield self._state
                    continue

                self._drive_heading_step()
                yield self._state
                continue

            if self._state == S5_BUMP_CAPTURE:
                self._viL.put(0.0)
                self._viR.put(0.0)
                self._start_dist = self._avg_dist()
                self._state = S6_BACKUP
                yield self._state
                continue

            if self._state == S6_BACKUP:
                self._viL.put(-self._backup_speed)
                self._viR.put(-self._backup_speed)
                if abs(self._avg_dist() - self._start_dist) >= self._backup_distance:
                    self._set_target(self._east_heading)
                    self._state = S7_TURN_EAST
                yield self._state
                continue

            if self._state == S7_TURN_EAST:
                if self._turn_step():
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S8_RESUME_LINE
                yield self._state
                continue

            if self._state == S8_RESUME_LINE:
                self._viL.put(0.0)
                self._viR.put(0.0)
                self._recoverGo.put(False)
                if self._blackDetectEnable is not None:
                    self._blackDetectEnable.put(True)
                self._lineGo.put(True)
                yield self._state
                self._state = S0_IDLE
                continue

            if self._state == S9_FINAL_CAPTURE:
                self._viL.put(0.0)
                self._viR.put(0.0)

                if self._blackStopGo is not None:
                    self._blackStopGo.put(False)

                self._reset_xy(self._black_reset_x, self._black_reset_y)

                self._set_target(float(self._imu_h.get()))
                self._start_dist = self._avg_dist()
                self._state = S10_FINAL_FORWARD
                yield self._state
                continue

            if self._state == S10_FINAL_FORWARD:
                self._drive_heading_step()
                if abs(self._avg_dist() - self._start_dist) >= self._final_preturn:
                    self._set_target(self._final_turn_heading)
                    self._state = S11_FINAL_TURN
                yield self._state
                continue

            if self._state == S11_FINAL_TURN:
                if self._turn_step():
                    self._state = S12_FINAL_DRIVE_XY1
                yield self._state
                continue

            if self._state == S12_FINAL_DRIVE_XY1:
                if self._drive_to_xy_step(self._final_target_x, self._final_target_y) <= self._final_target_tol:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._start_dist = self._avg_dist()
                    self._state = S13_BACKUP_BEFORE_XY2
                yield self._state
                continue

            if self._state == S13_BACKUP_BEFORE_XY2:
                self._viL.put(-self._backup_speed)
                self._viR.put(-self._backup_speed)
                if abs(self._avg_dist() - self._start_dist) >= self._final_target2_backup:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._set_target(self._final_target2_heading)
                    self._state = S14_TURN_90
                yield self._state
                continue

            if self._state == S14_TURN_90:
                if self._turn_step():
                    self._state = S15_FINAL_DRIVE_XY2
                yield self._state
                continue

            if self._state == S15_FINAL_DRIVE_XY2:
                if self._drive_to_xy_step(self._final_target2_x, self._final_target2_y) <= self._final_target_tol:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S16_FINAL_DRIVE_XY3
                yield self._state
                continue

            if self._state == S16_FINAL_DRIVE_XY3:
                if self._drive_to_xy_step(self._final_target3_x, self._final_target3_y) <= self._final_target_tol:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S17_RESUME_LINE_FINAL
                yield self._state
                continue

            if self._state == S17_RESUME_LINE_FINAL:
                self._viL.put(0.0)
                self._viR.put(0.0)

                if self._blackDetectEnable is not None:
                    self._blackDetectEnable.put(False)
                if self._kpline is not None:
                    self._kpline.put(self._final_line_kp)
                if self._kiline is not None:
                    self._kiline.put(self._final_line_ki)
                if self._baseLineV is not None:
                    self._baseLineV.put(self._final_line_speed)

                self._recoverGo.put(False)
                self._lineGo.put(True)
                yield self._state
                self._state = S0_IDLE
                continue

            if self._state == S18_RETURN_CAPTURE:
                self._viL.put(0.0)
                self._viR.put(0.0)
                self._reset_xy(self._return_reset_x, self._return_reset_y)
                self._recoverGo.put(False)
                self._state = S19_RETURN_DRIVE_MID1
                yield self._state
                continue

            if self._state == S19_RETURN_DRIVE_MID1:
                if self._drive_to_xy_step(self._return_mid1_x, self._return_mid1_y) <= self._final_target_tol:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S20_RETURN_DRIVE_MID2
                yield self._state
                continue

            if self._state == S20_RETURN_DRIVE_MID2:
                if self._drive_to_xy_step(self._return_mid2_x, self._return_mid2_y) <= self._final_target_tol:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S21_RETURN_DRIVE_XY1
                yield self._state
                continue

            if self._state == S21_RETURN_DRIVE_XY1:
                if self._drive_to_xy_step(self._return_target1_x, self._return_target1_y) <= self._final_target_tol:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S22_RETURN_DRIVE_XY2
                yield self._state
                continue

            if self._state == S22_RETURN_DRIVE_XY2:
                if self._drive_to_xy_step(self._return_target2_x, self._return_target2_y) <= self._final_target_tol:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._spin_start_ms = ticks_ms()
                    self._state = S23_FINAL_SPIN
                yield self._state
                continue

            if self._state == S23_FINAL_SPIN:
                self._lineGo.put(False)
                self._recoverGo.put(False)
                self._viL.put(self._final_spin_speed)
                self._viR.put(-self._final_spin_speed)

                if ticks_diff(ticks_ms(), self._spin_start_ms) >= self._final_spin_time_ms:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S24_FINAL_STOP

                yield self._state
                continue

            if self._state == S24_FINAL_STOP:
                self._viL.put(0.0)
                self._viR.put(0.0)
                self._lineGo.put(False)
                self._recoverGo.put(False)
                yield self._state
                self._state = S0_IDLE
=======
import micropython
import math
from utime import ticks_ms, ticks_diff
from heading_nav import HeadingNav

S0_IDLE = micropython.const(0)
S1_CAPTURE = micropython.const(1)
S2_FORWARD = micropython.const(2)
S3_TURN = micropython.const(3)
S4_DRIVE = micropython.const(4)
S5_BUMP_CAPTURE = micropython.const(5)
S6_BACKUP = micropython.const(6)
S7_TURN_EAST = micropython.const(7)
S8_RESUME_LINE = micropython.const(8)
S9_FINAL_CAPTURE = micropython.const(9)
S10_FINAL_FORWARD = micropython.const(10)
S11_FINAL_TURN = micropython.const(11)
S12_FINAL_DRIVE_XY1 = micropython.const(12)
S13_BACKUP_BEFORE_XY2 = micropython.const(13)
S14_TURN_90 = micropython.const(14)
S15_FINAL_DRIVE_XY2 = micropython.const(15)
S16_FINAL_DRIVE_XY3 = micropython.const(16)
S17_RESUME_LINE_FINAL = micropython.const(17)
S18_RETURN_CAPTURE = micropython.const(18)
S19_RETURN_DRIVE_MID1 = micropython.const(19)
S20_RETURN_DRIVE_MID2 = micropython.const(20)
S21_RETURN_DRIVE_XY1 = micropython.const(21)
S22_RETURN_DRIVE_XY2 = micropython.const(22)
S23_FINAL_SPIN = micropython.const(23)
S24_FINAL_STOP = micropython.const(24)


class task_recover:
    def __init__(self, lineGo, recoverGo, lineLostCount,
                 imu_heading_rel, vi_L, vi_R, sL_m_share, sR_m_share,
                 target_heading_share=None, bump_pin=None,
                 blackStopGo=None, blackDetectEnable=None,
                 posX_share=None, posY_share=None,
                 kpline_share=None, kiline_share=None,
                 baseLineV_share=None,
                 est_reset_go=None, initX_share=None, initY_share=None,
                 black_reset_x_m=1.175, black_reset_y_m=0.225,
                 return_reset_x_m=0.400, return_reset_y_m=0.500,
                 fwd_distance_m=0.100, base_fwd=0.08, turn_speed=0.08,
                 kp_heading=0.01, heading_tol=2.0,
                 recover_heading_deg=140.0, drive_heading_deg=180.0,
                 backup_distance_m=0.005, east_heading_deg=110.0,
                 backup_speed=0.06, final_preturn_m=0.01,
                 final_turn_heading_deg=50.0,
                 final_target_x_m=1.550, final_target_y_m=0.150,
                 final_target2_backup_m=0.030,
                 final_target2_heading_deg=90.0,
                 final_target2_x_m=1.300, final_target2_y_m=0.075,
                 final_target3_x_m=1.150, final_target3_y_m=0.050,
                 final_target_tol_m=0.030,
                 final_line_kp=16.0, final_line_ki=0.2, final_line_speed=0.045,
                 return_mid1_x_m=0.900, return_mid1_y_m=0.375,
                 return_mid2_x_m=0.950, return_mid2_y_m=0.600,
                 return_target1_x_m=0.150, return_target1_y_m=0.550,
                 return_target2_x_m=0.050, return_target2_y_m=0.800):

        self._state = S0_IDLE

        self._lineGo = lineGo
        self._recoverGo = recoverGo
        self._lineLostCount = lineLostCount

        self._imu_h = imu_heading_rel
        self._viL = vi_L
        self._viR = vi_R
        self._sL = sL_m_share
        self._sR = sR_m_share

        self._target_heading = target_heading_share
        self._bump_pin = bump_pin
        self._blackStopGo = blackStopGo
        self._blackDetectEnable = blackDetectEnable

        self._posX = posX_share
        self._posY = posY_share

        self._kpline = kpline_share
        self._kiline = kiline_share
        self._baseLineV = baseLineV_share

        self._est_reset_go = est_reset_go
        self._initX = initX_share
        self._initY = initY_share
        self._black_reset_x = float(black_reset_x_m)
        self._black_reset_y = float(black_reset_y_m)
        self._return_reset_x = float(return_reset_x_m)
        self._return_reset_y = float(return_reset_y_m)

        self._nav = HeadingNav(
            turn_speed=turn_speed,
            drive_speed=base_fwd,
            kp_drive=kp_heading,
            heading_tol=heading_tol
        )

        self._fwd_distance = float(fwd_distance_m)
        self._recover_heading = float(recover_heading_deg)
        self._drive_heading = float(drive_heading_deg)
        self._backup_distance = float(backup_distance_m)
        self._east_heading = float(east_heading_deg)
        self._backup_speed = float(backup_speed)

        self._final_preturn = float(final_preturn_m)
        self._final_turn_heading = float(final_turn_heading_deg)
        self._final_target_x = float(final_target_x_m)
        self._final_target_y = float(final_target_y_m)
        self._final_target2_backup = float(final_target2_backup_m)
        self._final_target2_heading = float(final_target2_heading_deg)
        self._final_target2_x = float(final_target2_x_m)
        self._final_target2_y = float(final_target2_y_m)
        self._final_target3_x = float(final_target3_x_m)
        self._final_target3_y = float(final_target3_y_m)
        self._final_target_tol = float(final_target_tol_m)

        self._final_line_kp = float(final_line_kp)
        self._final_line_ki = float(final_line_ki)
        self._final_line_speed = float(final_line_speed)

        self._return_mid1_x = float(return_mid1_x_m)
        self._return_mid1_y = float(return_mid1_y_m)
        self._return_mid2_x = float(return_mid2_x_m)
        self._return_mid2_y = float(return_mid2_y_m)
        self._return_target1_x = float(return_target1_x_m)
        self._return_target1_y = float(return_target1_y_m)
        self._return_target2_x = float(return_target2_x_m)
        self._return_target2_y = float(return_target2_y_m)

        self._start_dist = 0.0
        self._final_spin_speed = 0.4
        self._final_spin_time_ms = 700
        self._spin_start_ms = 0

    def _avg_dist(self):
        return 0.5 * (float(self._sL.get()) + float(self._sR.get()))

    def _set_target(self, ang):
        self._nav.set_goal_heading(ang)
        if self._target_heading is not None:
            self._target_heading.put(self._nav.goal_angle)

    def _turn_step(self):
        h = float(self._imu_h.get())
        vL, vR, done = self._nav.turn_to_heading(h)
        self._viL.put(vL)
        self._viR.put(vR)
        return done

    def _drive_heading_step(self):
        h = float(self._imu_h.get())
        vL, vR = self._nav.go_straight(h)
        self._viL.put(vL)
        self._viR.put(vR)

    def _drive_to_xy_step(self, xg, yg):
        x = 0.0 if self._posX is None else float(self._posX.get())
        y = 0.0 if self._posY is None else float(self._posY.get())
        dx = xg - x
        dy = yg - y
        dist = math.sqrt(dx * dx + dy * dy)
        self._set_target(math.degrees(math.atan2(-dy, dx)))
        self._drive_heading_step()
        return dist

    def _reset_xy(self, x, y):
        if self._initX is not None:
            self._initX.put(x)
        if self._initY is not None:
            self._initY.put(y)
        if self._posX is not None:
            self._posX.put(x)
        if self._posY is not None:
            self._posY.put(y)
        if self._est_reset_go is not None:
            self._est_reset_go.put(True)

    def run(self):
        while True:
            if self._state == S0_IDLE:
                if self._blackStopGo is not None and self._blackStopGo.get():
                    self._state = S9_FINAL_CAPTURE
                    yield self._state
                    continue

                if self._recoverGo.get() and (not self._lineGo.get()):
                    if self._lineLostCount is not None and int(self._lineLostCount.get()) >= 2:
                        self._state = S18_RETURN_CAPTURE
                    else:
                        self._state = S1_CAPTURE

                yield self._state
                continue

            if self._state == S1_CAPTURE:
                self._set_target(float(self._imu_h.get()))
                self._start_dist = self._avg_dist()
                self._state = S2_FORWARD
                yield self._state
                continue

            if self._state == S2_FORWARD:
                self._drive_heading_step()
                if abs(self._avg_dist() - self._start_dist) >= self._fwd_distance:
                    self._set_target(self._recover_heading)
                    self._state = S3_TURN
                yield self._state
                continue

            if self._state == S3_TURN:
                if self._turn_step():
                    self._set_target(self._drive_heading)
                    self._recoverGo.put(False)
                    self._state = S4_DRIVE
                yield self._state
                continue

            if self._state == S4_DRIVE:
                if self._bump_pin is not None and self._bump_pin.value() == 0:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S5_BUMP_CAPTURE
                    yield self._state
                    continue

                self._drive_heading_step()
                yield self._state
                continue

            if self._state == S5_BUMP_CAPTURE:
                self._viL.put(0.0)
                self._viR.put(0.0)
                self._start_dist = self._avg_dist()
                self._state = S6_BACKUP
                yield self._state
                continue

            if self._state == S6_BACKUP:
                self._viL.put(-self._backup_speed)
                self._viR.put(-self._backup_speed)
                if abs(self._avg_dist() - self._start_dist) >= self._backup_distance:
                    self._set_target(self._east_heading)
                    self._state = S7_TURN_EAST
                yield self._state
                continue

            if self._state == S7_TURN_EAST:
                if self._turn_step():
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S8_RESUME_LINE
                yield self._state
                continue

            if self._state == S8_RESUME_LINE:
                self._viL.put(0.0)
                self._viR.put(0.0)
                self._recoverGo.put(False)
                if self._blackDetectEnable is not None:
                    self._blackDetectEnable.put(True)
                self._lineGo.put(True)
                yield self._state
                self._state = S0_IDLE
                continue

            if self._state == S9_FINAL_CAPTURE:
                self._viL.put(0.0)
                self._viR.put(0.0)

                if self._blackStopGo is not None:
                    self._blackStopGo.put(False)

                self._reset_xy(self._black_reset_x, self._black_reset_y)

                self._set_target(float(self._imu_h.get()))
                self._start_dist = self._avg_dist()
                self._state = S10_FINAL_FORWARD
                yield self._state
                continue

            if self._state == S10_FINAL_FORWARD:
                self._drive_heading_step()
                if abs(self._avg_dist() - self._start_dist) >= self._final_preturn:
                    self._set_target(self._final_turn_heading)
                    self._state = S11_FINAL_TURN
                yield self._state
                continue

            if self._state == S11_FINAL_TURN:
                if self._turn_step():
                    self._state = S12_FINAL_DRIVE_XY1
                yield self._state
                continue

            if self._state == S12_FINAL_DRIVE_XY1:
                if self._drive_to_xy_step(self._final_target_x, self._final_target_y) <= self._final_target_tol:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._start_dist = self._avg_dist()
                    self._state = S13_BACKUP_BEFORE_XY2
                yield self._state
                continue

            if self._state == S13_BACKUP_BEFORE_XY2:
                self._viL.put(-self._backup_speed)
                self._viR.put(-self._backup_speed)
                if abs(self._avg_dist() - self._start_dist) >= self._final_target2_backup:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._set_target(self._final_target2_heading)
                    self._state = S14_TURN_90
                yield self._state
                continue

            if self._state == S14_TURN_90:
                if self._turn_step():
                    self._state = S15_FINAL_DRIVE_XY2
                yield self._state
                continue

            if self._state == S15_FINAL_DRIVE_XY2:
                if self._drive_to_xy_step(self._final_target2_x, self._final_target2_y) <= self._final_target_tol:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S16_FINAL_DRIVE_XY3
                yield self._state
                continue

            if self._state == S16_FINAL_DRIVE_XY3:
                if self._drive_to_xy_step(self._final_target3_x, self._final_target3_y) <= self._final_target_tol:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S17_RESUME_LINE_FINAL
                yield self._state
                continue

            if self._state == S17_RESUME_LINE_FINAL:
                self._viL.put(0.0)
                self._viR.put(0.0)

                if self._blackDetectEnable is not None:
                    self._blackDetectEnable.put(False)
                if self._kpline is not None:
                    self._kpline.put(self._final_line_kp)
                if self._kiline is not None:
                    self._kiline.put(self._final_line_ki)
                if self._baseLineV is not None:
                    self._baseLineV.put(self._final_line_speed)

                self._recoverGo.put(False)
                self._lineGo.put(True)
                yield self._state
                self._state = S0_IDLE
                continue

            if self._state == S18_RETURN_CAPTURE:
                self._viL.put(0.0)
                self._viR.put(0.0)
                self._reset_xy(self._return_reset_x, self._return_reset_y)
                self._recoverGo.put(False)
                self._state = S19_RETURN_DRIVE_MID1
                yield self._state
                continue

            if self._state == S19_RETURN_DRIVE_MID1:
                if self._drive_to_xy_step(self._return_mid1_x, self._return_mid1_y) <= self._final_target_tol:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S20_RETURN_DRIVE_MID2
                yield self._state
                continue

            if self._state == S20_RETURN_DRIVE_MID2:
                if self._drive_to_xy_step(self._return_mid2_x, self._return_mid2_y) <= self._final_target_tol:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S21_RETURN_DRIVE_XY1
                yield self._state
                continue

            if self._state == S21_RETURN_DRIVE_XY1:
                if self._drive_to_xy_step(self._return_target1_x, self._return_target1_y) <= self._final_target_tol:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S22_RETURN_DRIVE_XY2
                yield self._state
                continue

            if self._state == S22_RETURN_DRIVE_XY2:
                if self._drive_to_xy_step(self._return_target2_x, self._return_target2_y) <= self._final_target_tol:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._spin_start_ms = ticks_ms()
                    self._state = S23_FINAL_SPIN
                yield self._state
                continue

            if self._state == S23_FINAL_SPIN:
                self._lineGo.put(False)
                self._recoverGo.put(False)
                self._viL.put(self._final_spin_speed)
                self._viR.put(-self._final_spin_speed)

                if ticks_diff(ticks_ms(), self._spin_start_ms) >= self._final_spin_time_ms:
                    self._viL.put(0.0)
                    self._viR.put(0.0)
                    self._state = S24_FINAL_STOP

                yield self._state
                continue

            if self._state == S24_FINAL_STOP:
                self._viL.put(0.0)
                self._viR.put(0.0)
                self._lineGo.put(False)
                self._recoverGo.put(False)
                yield self._state
                self._state = S0_IDLE
>>>>>>> 441140e67c8ea020b13a2f9031ea66f0de0488d8:Firmware/task_recover.py
                continue