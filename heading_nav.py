class HeadingNav:
    def __init__(self, turn_speed=0.08, drive_speed=0.08, kp_drive=0.08, heading_tol=2.0):
        self.turn_speed = float(turn_speed)
        self.drive_speed = float(drive_speed)
        self.kp_drive = float(kp_drive)
        self.heading_tol = float(heading_tol)
        self.goal_angle = 0.0

    def _wrap_deg(self, ang):
        while ang >= 180.0:
            ang -= 360.0
        while ang < -180.0:
            ang += 360.0
        return ang

    def _clamp(self, x, lo, hi):
        if x < lo:
            return lo
        if x > hi:
            return hi
        return x

    def set_goal_heading(self, goal_heading):
        self.goal_angle = self._wrap_deg(float(goal_heading))
        return self.goal_angle

    def turn_to_heading(self, current_heading):
        err = self._wrap_deg(self.goal_angle - float(current_heading))

        if abs(err) <= self.heading_tol:
            return 0.0, 0.0, True

        cmd = self.turn_speed
        if err < 0.0:
            cmd = -cmd

        return cmd, -cmd, False

    def go_straight(self, current_heading):
        err = self._wrap_deg(self.goal_angle - float(current_heading))
        corr = self.kp_drive * err
        corr = self._clamp(corr, -0.25, 0.25)

        vL = self.drive_speed + corr
        vR = self.drive_speed - corr
        return vL, vR