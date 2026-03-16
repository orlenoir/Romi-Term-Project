import gc
from motor_driver import motor_driver
from encoder import encoder
from task_motor import task_motor
from task_user import task_user
from task_share import Share
from cotask import Task, task_list
from pyb import Timer, Pin, I2C, ADC
from line_sensor import QTR8A
from task_line import task_line
from IMU_driver import BNO055
from IMU_task import task_imu
from task_estimator import task_estimator
from Matrices import Ad, Bd
gc.collect()

from task_recover import task_recover

gc.collect()

tim3 = Timer(3, freq=20000)
tim4 = Timer(4, freq=20000)

kp = Share('f')
ki = Share('f')
kp.put(0.5)
ki.put(0.5)

baseLineV = Share('f')
baseLineV.put(0.1)

pwm_L = tim3.channel(1, Timer.PWM, pin=Pin.cpu.C6)
pwm_R = tim4.channel(2, Timer.PWM, pin=Pin.cpu.B7)

leftMotor = motor_driver(pwm_L, Pin.cpu.B10, Pin.cpu.B11)
rightMotor = motor_driver(pwm_R, Pin.cpu.H0, Pin.cpu.H1)

leftEncoder = encoder(1, Pin.cpu.A8, Pin.cpu.A9)
rightEncoder = encoder(2, Pin.cpu.A0, Pin.cpu.A1)

adc_pins = ['PC0', 'PC1', 'PC2', 'PC3', 'PA4', 'PC5', 'PA7', 'PA6']
adcs = [ADC(Pin(p)) for p in adc_pins]

bump_pin = Pin(Pin.cpu.C8, mode=Pin.IN, pull=Pin.PULL_UP)

i2c = I2C(1, I2C.CONTROLLER, baudrate=100000)
imu = BNO055(i2c, addr=0x28, rst_pin=Pin.cpu.B5, do_reset=False)
imu.set_mode(BNO055.MODE_IMUPLUS)

lineSensor = QTR8A(adcs, settle_ms=0)

gc.collect()

lineGo = Share('B')
lineGo.put(False)

recoverGo = Share('B')
recoverGo.put(False)

blackStopGo = Share('B')
blackStopGo.put(False)

blackDetectEnable = Share('B')
blackDetectEnable.put(False)

kpline = Share('f')
kiline = Share('f')
kpline.put(1.0)
kiline.put(0.5)

imu_heading = Share('f')
imu_yawrate = Share('f')

uL_V = Share('f')
uR_V = Share('f')

sL_m = Share('f')
sR_m = Share('f')

xhat0 = Share('f')
xhat1 = Share('f')
xhat2 = Share('f')
xhat3 = Share('f')

posX = Share('f')
posY = Share('f')
posX.put(0.100)
posY.put(0.800)

estResetGo = Share('B')
estResetGo.put(False)

estInitX = Share('f')
estInitY = Share('f')
estInitX.put(0.100)
estInitY.put(0.800)

imu_heading_abs = Share('f')
imu_heading_abs.put(0.0)

imu_heading0 = Share('f')
imu_heading0.put(0.0)

vi_L = Share('f')
vi_R = Share('f')
vi_L.put(0.0)
vi_R.put(0.0)

leftMotorGo = Share('B')
rightMotorGo = Share('B')
leftMotorGo.put(False)
rightMotorGo.put(False)

lineLostCount = Share('L')
lineLostCount.put(0)

gc.collect()

lineTask = task_line(
    lineSensor, vi_L, vi_R,
    lineGo=lineGo, kpline=kpline, kiline=kiline,
    base_vi=baseLineV,
    timeQ=None, centroidQ=None,
    errorQ=None, deltaQ=None, integralQ=None,
    lineLostCount=lineLostCount, recoverGo=recoverGo,
    imu_heading_abs_share=imu_heading_abs, imu_heading0_share=imu_heading0,
    blackStopGo=blackStopGo, blackDetectEnable=blackDetectEnable,
    black_threshold=1800,
    posY_share=posY,
    final_line_exit_y_m=0.400,
    return_line_lost_threshold=400,
    first_line_lost_threshold=450
)

gc.collect()

userTask = task_user(
    lineSensor,
    lineGo, kpline, kiline,
    kp, ki,
    vi_L, vi_R,
    leftMotorGo, rightMotorGo, baseLineV,
    errorQ=None, deltaQ=None, integralQ=None,
    timeQ=None, centroidQ=None,
    imu_heading_share=imu_heading,
    imu_yawrate_share=imu_yawrate,
    imu_calraw_share=None,
    xhat_shares=None,
    posX_share=posX,
    posY_share=posY,
    lineLostCount=lineLostCount,
    imu_heading_abs_share=imu_heading_abs,
    imu_heading0_share=imu_heading0,
    imu_target_heading_share=None,
    est_reset_go=estResetGo,
    est_init_x_share=estInitX,
    est_init_y_share=estInitY,
    recoverGo=recoverGo,
    blackStopGo=blackStopGo,
    blackDetectEnable=blackDetectEnable,
    user_button_pin=Pin.cpu.C13,
    user_button_active_value=1
)

gc.collect()

estTask = task_estimator(
    Ad=Ad, Bd=Bd,
    uL_V_share=uL_V, uR_V_share=uR_V,
    sL_m_share=sL_m, sR_m_share=sR_m,
    psi_deg_share=imu_heading,
    psidot_dps_share=imu_yawrate,
    xhat_shares=(xhat0, xhat1, xhat2, xhat3),
    posX_share=posX, posY_share=posY,
    period_ms=20,
    lineLostCount=lineLostCount,
    reset_pos_go=estResetGo,
    initX_share=estInitX,
    initY_share=estInitY
)

gc.collect()

imuTask = task_imu(
    imu, imu_heading, imu_yawrate,
    calraw_share=None,
    lineLostCount=lineLostCount,
    heading_abs_share=imu_heading_abs,
    heading0_share=imu_heading0
)

leftMotorTask = task_motor(
    leftMotor, leftEncoder,
    leftMotorGo, None, None, kp, ki, vi_L,
    uV_share=uL_V, s_m_share=sL_m, Vbatt=6.0
)

rightMotorTask = task_motor(
    rightMotor, rightEncoder,
    rightMotorGo, None, None, kp, ki, vi_R,
    uV_share=uR_V, s_m_share=sR_m, Vbatt=6.0
)

gc.collect()

recoverTask = task_recover(
    lineGo=lineGo,
    recoverGo=recoverGo,
    lineLostCount=lineLostCount,
    imu_heading_rel=imu_heading,
    vi_L=vi_L,
    vi_R=vi_R,
    sL_m_share=sL_m,
    sR_m_share=sR_m,
    target_heading_share=None,
    bump_pin=bump_pin,
    blackStopGo=blackStopGo,
    blackDetectEnable=blackDetectEnable,
    posX_share=posX,
    posY_share=posY,
    kpline_share=kpline,
    kiline_share=kiline,
    baseLineV_share=baseLineV,
    est_reset_go=estResetGo,
    initX_share=estInitX,
    initY_share=estInitY,
    black_reset_x_m=1.175,
    black_reset_y_m=0.225,
    fwd_distance_m=0.100,
    base_fwd=0.08,
    turn_speed=0.08,
    kp_heading=0.01,
    heading_tol=2.0,
    recover_heading_deg=140.0,
    drive_heading_deg=180.0,
    backup_distance_m=0.005,
    east_heading_deg=110.0,
    backup_speed=0.06,
    final_preturn_m=0.01,
    final_turn_heading_deg=50.0,
    final_target_x_m=1.550,
    final_target_y_m=0.150,
    final_target2_backup_m=0.030,
    final_target2_heading_deg=90.0,
    final_target2_x_m=1.300,
    final_target2_y_m=0.075,
    final_target3_x_m=1.150,
    final_target3_y_m=0.050,
    final_target_tol_m=0.030,
    final_line_kp=16.0,
    final_line_ki=0.2,
    final_line_speed=0.045,
    return_mid1_x_m=0.900,
    return_mid1_y_m=0.375,
    return_mid2_x_m=0.950,
    return_mid2_y_m=0.600,
    return_target1_x_m=0.150,
    return_target1_y_m=0.550,
    return_target2_x_m=0.050,
    return_target2_y_m=0.800
)

gc.collect()

task_list.append(Task(estTask.run, priority=3, period=20, profile=False))
task_list.append(Task(lineTask.run, priority=2, period=20, profile=False))
task_list.append(Task(imuTask.run, priority=2, period=20, profile=False))
task_list.append(Task(recoverTask.run, priority=2, period=20, profile=False))
task_list.append(Task(leftMotorTask.run, priority=1, period=50, profile=False))
task_list.append(Task(rightMotorTask.run, priority=1, period=50, profile=False))
task_list.append(Task(userTask.run, priority=0, period=20, profile=False))

gc.collect()

while True:
    try:
        task_list.pri_sched()
    except KeyboardInterrupt:
        break