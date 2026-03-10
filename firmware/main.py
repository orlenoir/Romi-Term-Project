from motor_driver import motor_driver
from encoder      import encoder
from task_motor   import task_motor
from task_user    import task_user
from task_share   import Share, Queue, show_all
from cotask       import Task, task_list
from gc           import collect
from pyb          import Timer, Pin, I2C
from pyb          import repl_uart
from line_sensor import QTR8A
from task_line   import task_line
from pyb import ADC, Pin
from IMU_driver import BNO055
from IMU_task import task_imu
from task_estimator import task_estimator
from matrices import Ad, Bd
tim3 = Timer(3, freq=20000)
tim4 = Timer(4, freq=20000)


kp   = Share('f', name="kp")
ki   = Share('f', name="ki")
baseLineV = Share('f', name="Line Base V")
baseLineV.put(0.0)

pwm_L = tim3.channel(1, Timer.PWM, pin=Pin.cpu.C6)
pwm_R = tim4.channel(2, Timer.PWM, pin=Pin.cpu.B7)

leftMotor  = motor_driver(pwm_L, Pin.cpu.B10,  Pin.cpu.B11)
rightMotor = motor_driver(pwm_R, Pin.cpu.H0, Pin.cpu.H1)

leftEncoder  = encoder(1, Pin.cpu.A8, Pin.cpu.A9)
rightEncoder = encoder(2, Pin.cpu.A0, Pin.cpu.A1)



adc_pins = ['PC0','PC1','PC2','PC3','PA4','PC5','PA7','PA6']

adcs = [ADC(Pin(p)) for p in adc_pins]  

i2c = I2C(1, I2C.CONTROLLER, baudrate=100000)
imu = BNO055(i2c, addr=0x28, rst_pin=Pin.cpu.B5, do_reset=False) #change reset to true later
imu.set_mode(BNO055.MODE_IMUPLUS)

lineSensor = QTR8A(adcs, settle_ms=0)

lineGo   = Share("B", name="Line Follow Enable")
kpline   = Share("f", name="Line Kp")
kiline   = Share("f", name="Line Ki")
imu_heading = Share('f', name="IMU heading deg")
imu_yawrate = Share('f', name="IMU yaw rate dps")
imu_calraw  = Share('B', name="IMU calib raw")


# --- Motor voltage shares (new) ---
uL_V = Share('f', name="uL volts")
uR_V = Share('f', name="uR volts")

# --- Wheel displacement shares (new) ---
sL_m = Share('f', name="sL meters")
sR_m = Share('f', name="sR meters")

# --- Estimated state outputs (new) ---
xhat0 = Share('f', name="xhat sL")
xhat1 = Share('f', name="xhat sR")
xhat2 = Share('f', name="xhat psi(rad)")
xhat3 = Share('f', name="xhat psidot(rad/s)")

posX = Share('f', name="posX m")
posY = Share('f', name="posY m")

timeLine = Queue("L", 500, name="Line Time")
centLine = Queue("f", 500, name="Line Centroid")
errLine  = Queue("f", 500, name="Line Error")
dvLine   = Queue("f", 500, name="Line DeltaV")
intLine  = Queue("f", 500, name="Line Integral")

lineGo.put(False)
kpline.put(0.00)
kiline.put(0.00)

vi_L = Share('f', name="vi_L")
vi_R = Share('f', name="vi_R")

vi_L.put(0.0)
vi_R.put(0.0)

lineTask = task_line(lineSensor, vi_L, vi_R,
                     lineGo=lineGo, kpline=kpline, kiline=kiline,
                     base_vi=baseLineV,
                     timeQ=timeLine, centroidQ=centLine,
                     errorQ=errLine, deltaQ=dvLine, integralQ=intLine) 

leftMotorGo    = Share("B", name="Left Mot. Go Flag")
rightMotorGo   = Share("B", name="Right Mot. Go Flag")
leftMotorGo.put(True)
rightMotorGo.put(True)

dataValuesL    = Queue("f", 300, name="Left Velocity Buffer")
timeValuesL    = Queue("L", 300, name="Left Time Buffer")
dataValuesR    = Queue("f", 300, name="Right Velocity Buffer")
timeValuesR    = Queue("L", 300, name="Right Time Buffer")

userTask = task_user(lineSensor,
                     lineGo, kpline, kiline,
                     kp, ki,
                     vi_L, vi_R,
                     leftMotorGo, rightMotorGo, baseLineV,
                     errorQ=errLine, deltaQ=dvLine, integralQ=intLine,
                     timeQ=timeLine, centroidQ=centLine,
                     imu_heading_share=imu_heading,
                     imu_yawrate_share=imu_yawrate,
                     imu_calraw_share=imu_calraw,
                     xhat_shares=(xhat0, xhat1, xhat2, xhat3),
                     posX_share=posX,
                     posY_share=posY)

estTask = task_estimator(Ad=Ad, Bd=Bd,
    uL_V_share=uL_V, uR_V_share=uR_V,
    sL_m_share=sL_m, sR_m_share=sR_m,
    psi_deg_share=imu_heading,
    psidot_dps_share=imu_yawrate,
    xhat_shares=(xhat0, xhat1, xhat2, xhat3),
    posX_share=posX, posY_share=posY,
    period_ms=20
)

task_list.append(Task(estTask.run, name="Estimator Task",
                      priority=3, period=20, profile=True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority=0, period=20, profile=False))
task_list.append(Task(lineTask.run, name="Line Task",
                      priority=2, period=20, profile=True))

imuTask = task_imu(imu, imu_heading, imu_yawrate, calraw_share=imu_calraw)

task_list.append(Task(imuTask.run, name="IMU Task",
                      priority=2, period=20, profile=True))

leftMotorTask  = task_motor(leftMotor,  leftEncoder,
                            leftMotorGo, dataValuesL, timeValuesL, kp, ki, vi_L,
                            uV_share=uL_V, s_m_share=sL_m, Vbatt=6.0)

rightMotorTask = task_motor(rightMotor, rightEncoder,
                            rightMotorGo, dataValuesR, timeValuesR, kp, ki, vi_R,
                            uV_share=uR_V, s_m_share=sR_m, Vbatt=6.0)

task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority=1, period=50, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority=1, period=50, profile=True))

collect()

leftMotor.enable()
rightMotor.enable()

while True:
    try:
        task_list.pri_sched()

    except KeyboardInterrupt:
        print("Program Terminating")
        leftMotor.disable()
        rightMotor.disable()
        break

print("\n")
print(task_list)
print(show_all())