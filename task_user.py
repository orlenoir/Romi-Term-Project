from pyb import USB_VCP, Pin
import micropython

S0_INIT = micropython.const(0)
S1_CMD  = micropython.const(1)

class task_user:
    def __init__(self,
                 sensor,
                 lineGo, kpline, kiline,
                 kp_wheel, ki_wheel,
                 vi_L, vi_R,
                 leftMotorGo, rightMotorGo, baseLineV, timeQ=None, centroidQ=None,
                 errorQ=None, deltaQ=None, integralQ=None,
                 imu_heading_share=None, imu_yawrate_share=None, imu_calraw_share=None,
                 xhat_shares=None, posX_share=None, posY_share=None,
                 lineLostCount=None,
                 imu_heading_abs_share=None, imu_heading0_share=None,
                 imu_target_heading_share=None,
                 est_reset_go=None, est_init_x_share=None, est_init_y_share=None,
                 recoverGo=None,
                 blackStopGo=None, blackDetectEnable=None,
                 user_button_pin=None,
                 user_button_active_value=1):

        self._state = S0_INIT

        self._sensor = sensor

        self._lineGo  = lineGo
        self._kpline  = kpline
        self._kiline  = kiline

        self._kpw = kp_wheel
        self._kiw = ki_wheel

        self._viL = vi_L
        self._viR = vi_R

        self._leftGo  = leftMotorGo
        self._rightGo = rightMotorGo

        self._baseLineV = baseLineV

        self._imu_heading_abs = imu_heading_abs_share
        self._imu_heading0 = imu_heading0_share
        self._imu_target_heading = imu_target_heading_share

        self.posX_share = posX_share
        self.posY_share = posY_share
        self._lineLostCount = lineLostCount

        self._est_reset_go = est_reset_go
        self._est_init_x = est_init_x_share
        self._est_init_y = est_init_y_share

        self._recoverGo = recoverGo
        self._blackStopGo = blackStopGo
        self._blackDetectEnable = blackDetectEnable

        self._user_button = None
        self._user_button_enabled = False
        self._user_button_active_value = int(user_button_active_value)

        if user_button_pin is not None:
            try:
                self._user_button = Pin(user_button_pin, mode=Pin.IN)
                self._user_button_enabled = True
            except Exception:
                self._user_button = None
                self._user_button_enabled = False

        if self._user_button_enabled and self._user_button is not None:
            try:
                self._button_prev_pressed = (
                    int(self._user_button.value()) == self._user_button_active_value
                )
            except Exception:
                self._button_prev_pressed = False
                self._user_button_enabled = False
        else:
            self._button_prev_pressed = False

        self._ser = USB_VCP()

        self._saved_line_kp = 1.0
        self._saved_line_ki = 0.5
        self._saved_wheel_kp = 0.5
        self._saved_wheel_ki = 0.5
        self._saved_speed = 0.1
        self._saved_start_x = 0.100
        self._saved_start_y = 0.800

    def _print_values(self):
        vals = self._sensor.read_raw()
        self._ser.write(str(vals) + "\r\n")

    def _go_with_saved_settings(self):
        if self._lineGo is not None:
            self._lineGo.put(False)

        if self._recoverGo is not None:
            self._recoverGo.put(False)

        if self._blackStopGo is not None:
            self._blackStopGo.put(False)

        if self._blackDetectEnable is not None:
            self._blackDetectEnable.put(False)

        if self._lineLostCount is not None:
            self._lineLostCount.put(0)

        if self._leftGo is not None:
            self._leftGo.put(True)

        if self._rightGo is not None:
            self._rightGo.put(True)

        self._kpline.put(self._saved_line_kp)
        self._kiline.put(self._saved_line_ki)
        self._kpw.put(self._saved_wheel_kp)
        self._kiw.put(self._saved_wheel_ki)

        self._baseLineV.put(self._saved_speed)
        self._viL.put(self._saved_speed)
        self._viR.put(self._saved_speed)

        if self._imu_heading_abs is not None and self._imu_heading0 is not None:
            self._imu_heading0.put(float(self._imu_heading_abs.get()))

        if self._imu_target_heading is not None:
            self._imu_target_heading.put(0.0)

        if self._est_init_x is not None:
            self._est_init_x.put(self._saved_start_x)

        if self._est_init_y is not None:
            self._est_init_y.put(self._saved_start_y)

        if self._est_reset_go is not None:
            self._est_reset_go.put(True)

        if self.posX_share is not None:
            self.posX_share.put(self._saved_start_x)

        if self.posY_share is not None:
            self.posY_share.put(self._saved_start_y)

        if self._lineGo is not None:
            self._lineGo.put(True)

    def _button_pressed(self):
        if not self._user_button_enabled or self._user_button is None:
            return False
        try:
            return int(self._user_button.value()) == self._user_button_active_value
        except Exception:
            self._user_button_enabled = False
            return False

    def _robot_busy(self):
        if self._lineGo is not None and self._lineGo.get():
            return True
        if self._recoverGo is not None and self._recoverGo.get():
            return True
        return False

    def _check_hardware_start(self):
        if not self._user_button_enabled:
            return

        pressed = self._button_pressed()
        start_event = pressed and (not self._button_prev_pressed)
        self._button_prev_pressed = pressed

        if start_event and (not self._robot_busy()):
            self._go_with_saved_settings()

    def run(self):
        while True:
            self._check_hardware_start()

            if self._state == S0_INIT:
                self._state = S1_CMD

            elif self._state == S1_CMD:
                if self._ser.any():
                    cmd_b = self._ser.read(1)
                    if not cmd_b:
                        yield self._state
                        continue

                    try:
                        cmd = chr(cmd_b[0])
                    except Exception:
                        try:
                            cmd = cmd_b.decode()
                        except Exception:
                            yield self._state
                            continue

                    if cmd in ("\r", "\n"):
                        yield self._state
                        continue

                    if cmd in {"w", "W"}:
                        self._print_values()
                        self._sensor.calibrate_white(50)
                        self._print_values()

                    elif cmd in {"b", "B"}:
                        self._print_values()
                        self._sensor.calibrate_black(50)
                        self._print_values()

            yield self._state