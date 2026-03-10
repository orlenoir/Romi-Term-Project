from pyb import USB_VCP
import micropython

S0_INIT = micropython.const(0)
S1_CMD  = micropython.const(1)

UI_prompt = ">: "


class task_user:
    def __init__(self,
                 sensor,
                 lineGo, kpline, kiline,
                 kp_wheel, ki_wheel,
                 vi_L, vi_R,
                 leftMotorGo, rightMotorGo, baseLineV, timeQ=None, centroidQ=None,
                 errorQ=None, deltaQ=None, integralQ=None,
                 # NEW IMU share inputs (optional)
                 imu_heading_share=None, imu_yawrate_share=None, imu_calraw_share=None,
                 xhat_shares=None, posX_share=None, posY_share=None):

        self._state = 0

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

        # NEW
        self._errorQ = errorQ
        self._deltaQ = deltaQ
        self._integralQ = integralQ

        self._timeQ = timeQ
        self._centroidQ = centroidQ

        self._baseLineV = baseLineV

        
        self._imu_heading = imu_heading_share
        self._imu_yawrate = imu_yawrate_share
        self._imu_calraw  = imu_calraw_share

        self.xhat_shares = xhat_shares
        self.posX_share = posX_share
        self.posY_share = posY_share

        self._ser = USB_VCP()
        self._ser.write("UI STARTED (Lab 0x05 Line Following)\r\n")

    def _get_latest_from_queue(self, q):
        if q is None:
            return None
        try:
            if not q.any():
                return None
        except Exception:

            try:
                latest = q.get_nowait()
            except Exception:
                return None
            while True:
                try:
                    latest = q.get_nowait()
                except Exception:
                    break
            return latest

        latest = None
        while q.any():
            latest = q.get()
        return latest

    def getfloat(self):
        """Generator that reads a float from USB serial without blocking."""
        buf = ""
        digits = set("0123456789")
        term = {"\r", "\n"}

        while True:
            if self._ser.any():
                c = self._ser.read(1).decode()

                if c in digits:
                    self._ser.write(c)
                    buf += c

                elif c == "." and "." not in buf:

                    if buf == "":
                        buf = "0"
                        self._ser.write("0")
                    self._ser.write(".")
                    buf += "."

                elif c == "-" and len(buf) == 0:
                    self._ser.write(c)
                    buf += c

                elif c == "\x7f" and len(buf) > 0:
                    buf = buf[:-1]

                elif c in term:
                    self._ser.write("\r\n")

                    if buf in {"", "-", ".", "0."}:
                        return None

                    return float(buf)

            yield

    def _print_menu(self):
        self._ser.write("\r\n+-------------------------------------------------------------+\r\n")
        self._ser.write("| ME 405 Romi Line Following UI (Lab 0x05)                    |\r\n")
        self._ser.write("+---+---------------------------------------------------------+\r\n")
        self._ser.write("| h | Help menu                                               |\r\n")
        self._ser.write("| w | Calibrate WHITE (on white surface)                      |\r\n")
        self._ser.write("| b | Calibrate BLACK (on black line)                         |\r\n")
        self._ser.write("| f | Toggle line following ON/OFF                            |\r\n")
        self._ser.write("| p | Set line gains (Kp_line, Ki_line)                       |\r\n")
        self._ser.write("| k | Set wheel gains (Kp_wheel, Ki_wheel)                    |\r\n")
        self._ser.write("| v | Set forward speed (base speed for line follow)          |\r\n")
        self._ser.write("| t | Dump centroid vs time (CSV)                             |\r\n")
        self._ser.write("| d | Debug: print latest error/integral/dv                   |\r\n")
        self._ser.write("| x | STOP (0 speed + line follow OFF)                        |\r\n")
        self._ser.write("| i | Print IMU heading / yaw rate / cal status               |\r\n")
        self._ser.write("| e | Print estimator states (xhat and position)              |\r\n")
        self._ser.write("+---+---------------------------------------------------------+\r\n\r\n")

        self._ser.write("Status: line_follow=" + ("ON" if self._lineGo.get() else "OFF") + "\r\n")
        self._ser.write("Line gains:  Kp=" + str(self._kpline.get()) + "  Ki=" + str(self._kiline.get()) + "\r\n")
        self._ser.write("Wheel gains: Kp=" + str(self._kpw.get())   + "  Ki=" + str(self._kiw.get()) + "\r\n")
        self._ser.write("BaseV(line): " + str(self._baseLineV.get()) + " m/s\r\n")  # <-- NEW
        self._ser.write("vi_L=" + str(self._viL.get()) + "  vi_R=" + str(self._viR.get()) + "\r\n\r\n")
        self._ser.write(UI_prompt)

    def _stop_robot(self):
        self._lineGo.put(False)
        self._viL.put(0.0)
        self._viR.put(0.0)

    def run(self):
        while True:
            if self._state == S0_INIT:
                self._print_menu()
                self._state = S1_CMD

            elif self._state == S1_CMD:
                if self._ser.any():
                    cmd_b = self._ser.read(1)
                    if not cmd_b:
                        yield self._state
                        continue
                    cmd = cmd_b.decode()

    
                    if cmd in ("\r", "\n"):
                        yield self._state
                        continue

    
                    if cmd in {"h", "H"}:
                        self._ser.write(cmd + "\r\n")
                        self._state = S0_INIT

                    
                    elif cmd in {"w", "W"}:
                        self._ser.write(cmd + "\r\n")
                        self._ser.write("Calibrating WHITE... keep sensors on WHITE.\r\n")
                        self._sensor.calibrate_white(50)
                        self._ser.write("White calibration done.\r\n")
                        self._ser.write(UI_prompt)

               
                    elif cmd in {"b", "B"}:
                        self._ser.write(cmd + "\r\n")
                        self._ser.write("Calibrating BLACK... keep sensors on BLACK line.\r\n")
                        self._sensor.calibrate_black(50)
                        self._ser.write("Black calibration done.\r\n")
                        self._ser.write(UI_prompt)

                
                    elif cmd in {"f", "F"}:
                        self._ser.write(cmd + "\r\n")
                        new_state = not self._lineGo.get()
                        self._lineGo.put(new_state)
                        self._ser.write("Line follow is now " + ("ON" if new_state else "OFF") + ".\r\n")

                        self._ser.write(UI_prompt)


                    elif cmd in {"p", "P"}:
                        self._ser.write(cmd + "\r\n")
                        self._ser.write("Enter line proportional gain Kp_line:\r\n")
                        self._ser.write(UI_prompt)
                        kp = yield from self.getfloat()

                        self._ser.write("Enter line integral gain Ki_line:\r\n")
                        self._ser.write(UI_prompt)
                        ki = yield from self.getfloat()

                        if kp is not None:
                            self._kpline.put(kp)
                        if ki is not None:
                            self._kiline.put(ki)

                        self._ser.write("Line gains set: Kp_line=" + str(self._kpline.get()) +
                                        " Ki_line=" + str(self._kiline.get()) + "\r\n")
                        self._ser.write(UI_prompt)

                    elif cmd in {"k", "K"}:
                        self._ser.write(cmd + "\r\n")
                        self._ser.write("Enter wheel proportional gain Kp_wheel:\r\n")
                        self._ser.write(UI_prompt)
                        kp = yield from self.getfloat()

                        self._ser.write("Enter wheel integral gain Ki_wheel:\r\n")
                        self._ser.write(UI_prompt)
                        ki = yield from self.getfloat()

                        if kp is not None:
                            self._kpw.put(kp)
                        if ki is not None:
                            self._kiw.put(ki)

                        self._ser.write("Wheel gains set: Kp_wheel=" + str(self._kpw.get()) +
                                        " Ki_wheel=" + str(self._kiw.get()) + "\r\n")
                        self._ser.write(UI_prompt)

                    elif cmd in {"v", "V"}:
                        self._ser.write(cmd + "\r\n")
                        if self._lineGo.get():
                            self._ser.write("NOTE: line follow is ON; manual speed may be overwritten by line task.\r\n")

                        self._ser.write("Enter forward speed (m/s) to apply to BOTH wheels:\r\n")
                        self._ser.write(UI_prompt)
                        v = yield from self.getfloat()

                        if v is not None:
                            self._viL.put(v)
                            self._viR.put(v)
                            self._baseLineV.put(v)
                            self._ser.write("Manual speed set: vi_L=vi_R=" + str(v) + "\r\n")
                        else:
                            self._ser.write("No change.\r\n")

                        self._ser.write(UI_prompt)

                    elif cmd == 'd':
                        self._ser.write("\r\n----- DEBUG STATUS -----\r\n")

            
                        self._ser.write("lineGo       : " + str(self._lineGo.get()) + "\r\n")
                        self._ser.write("leftMotorGo  : " + str(self._leftGo.get()) + "\r\n")
                        self._ser.write("rightMotorGo : " + str(self._rightGo.get()) + "\r\n")

                   
                        self._ser.write("vi_L : " + str(self._viL.get()) + "\r\n")
                        self._ser.write("vi_R : " + str(self._viR.get()) + "\r\n")

                       
                        self._ser.write("Line Gains   : Kp=" + str(self._kpline.get()) +
                                        "  Ki=" + str(self._kiline.get()) + "\r\n")

                        self._ser.write("Wheel Gains  : Kp=" + str(self._kpw.get()) +
                                        "  Ki=" + str(self._kiw.get()) + "\r\n")

                        latest_err = self._get_latest_from_queue(self._errorQ)
                        latest_dv  = self._get_latest_from_queue(self._deltaQ)

                        self._ser.write("Line error (latest) : " + str(latest_err) + "\r\n")
                        latest_integral = self._get_latest_from_queue(self._integralQ)
                        self._ser.write("Line integral (self._integral): " + str(latest_integral) + "\r\n")
                        self._ser.write("Line dv (latest)     : " + str(latest_dv) + "\r\n")

                        self._ser.write("------------------------\r\n")
                        
                    elif cmd in {"x", "X"}:
                        self._ser.write(cmd + "\r\n")
                        self._stop_robot()
                        self._ser.write("STOPPED. line_follow=OFF, vi_L=vi_R=0\r\n")
                        self._ser.write(UI_prompt)

                    elif cmd in {"i", "I"}:
                        # IMU print command
                        self._ser.write(cmd + "\r\n")

                        # Heading
                        if self._imu_heading is None:
                            self._ser.write("IMU heading: (not configured)\r\n")
                        else:
                            try:
                                h = self._imu_heading.get()
                                self._ser.write("IMU heading (deg): " + str(h) + "\r\n")
                            except Exception as ex:
                                self._ser.write("IMU heading read error: " + str(ex) + "\r\n")

                        # Yaw rate
                        if self._imu_yawrate is None:
                            self._ser.write("IMU yaw rate: (not configured)\r\n")
                        else:
                            try:
                                y = self._imu_yawrate.get()
                                self._ser.write("IMU yaw rate (deg/s): " + str(y) + "\r\n")
                            except Exception as ex:
                                self._ser.write("IMU yaw rate read error: " + str(ex) + "\r\n")

                        # Calibration raw status (optional)
                        if self._imu_calraw is None:
                            self._ser.write("IMU cal status: (not configured)\r\n")
                        else:
                            try:
                                cal_raw = self._imu_calraw.get()
                                # Parse into component fields if cal_raw is a byte
                                try:
                                    b = int(cal_raw) & 0xFF
                                    sys_c = (b >> 6) & 0x03
                                    gyro_c = (b >> 4) & 0x03
                                    accel_c = (b >> 2) & 0x03
                                    mag_c = b & 0x03
                                    self._ser.write(
                                        "IMU cal (sys/g/acc/mag): {}/{}/{}/{}  (raw=0x{:02X})\r\n"
                                        .format(sys_c, gyro_c, accel_c, mag_c, b))
                                except Exception:
                                    self._ser.write("IMU cal raw: " + str(cal_raw) + "\r\n")
                            except Exception as ex:
                                self._ser.write("IMU cal read error: " + str(ex) + "\r\n")

                        self._ser.write(UI_prompt)
                    elif cmd == 'e':
                        if self.xhat_shares is not None:
                            x0 = self.xhat_shares[0].get()
                            x1 = self.xhat_shares[1].get()
                            x2 = self.xhat_shares[2].get()
                            x3 = self.xhat_shares[3].get()
                    
                            print("Estimator States:")
                            print(f"sL_hat: {x0:.6f}")
                            print(f"sR_hat: {x1:.6f}")
                            print(f"psi_hat (rad): {x2:.6f}")
                            print(f"psiDot_hat (rad/s): {x3:.6f}")
                    
                            if self.posX_share is not None and self.posY_share is not None:
                                print(f"posX: {self.posX_share.get():.6f}")
                                print(f"posY: {self.posY_share.get():.6f}")
                        else:
                            print("Estimator shares not connected.")
                    elif cmd in {"t", "T"}:
                        self._ser.write(cmd + "\r\n")

                        if (self._timeQ is None) or (self._centroidQ is None):
                            self._ser.write("ERROR: timeQ/centroidQ not connected.\r\n")
                            self._ser.write(UI_prompt)
                            yield self._state
                            continue

                        # Clear old samples so we output a fresh 7-second window
                        while self._timeQ.any() and self._centroidQ.any():
                            _ = self._timeQ.get()
                            _ = self._centroidQ.get()

                        self._ser.write("t_s,centroid\r\n")

                        first_t = None
                        while True:
                            # wait for a paired sample
                            if not (self._timeQ.any() and self._centroidQ.any()):
                                yield self._state
                                continue

                            t_us = self._timeQ.get()
                            c = self._centroidQ.get()

                            if first_t is None:
                                first_t = t_us

                            dt_us = t_us - first_t
                            if dt_us >= 7_000_000:
                                break

                            # Skip invalid centroid entries
                            if c != -1.0:
                                self._ser.write("{:.6f},{}\r\n".format(dt_us / 1_000_000.0, c))

                            yield self._state

                        self._ser.write("END\r\n")
                        self._ser.write(UI_prompt)

                        # Write to CSV on the board
                        fname = "centroid_7s.csv"
                        try:
                            with open(fname, "w") as f:
                                f.write("t_s,centroid\n")
                                for dt_us, c in samples:
                                    f.write("{:.6f},{}\n".format(dt_us / 1_000_000.0, c))
                            self._ser.write("Wrote {} rows to {}\r\n".format(len(samples), fname))
                        except Exception as ex:
                            self._ser.write("ERROR writing CSV: {}\r\n".format(ex))

                        self._ser.write(UI_prompt)
                        self._ser.write("END\r\n")
                        self._ser.write(UI_prompt)
                    else:

                        self._ser.write("\r\nUnknown cmd. Press 'h' for help.\r\n")
                        self._ser.write(UI_prompt)

            yield self._state