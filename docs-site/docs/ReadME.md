## main.py 
The main entry point of the program. This file initializes all hardware interfaces, creates shared variables, instantiates tasks, and starts the cooperative scheduler that runs the robot's control system. 

## cotask.py 
Implements the cooperative multitasking scheduler used in the project. It manages task priorities, timing, and execution so multiple control tasks can run concurrently without blocking each other. 

## task_share.py 
Provides the Share and Queue classes used for communication between tasks. These shared data structures allow tasks to safely exchange information such as sensor readings, velocity commands, and control flags. 

## task_user.py 
Implements the user interface task that communicates with the robot through the serial terminal. It processes commands such as calibration and starting the robot while updating shared variables used by other tasks. 

## task_motor.py 
Handles closed-loop control of each motor using encoder feedback. This task reads encoder data, applies the PI control algorithm, and sets the PWM duty cycle required to achieve the desired wheel velocity. 

## motor_driver.py 
A hardware abstraction class for controlling the Romi motors. It converts control signals from the motor task into PWM outputs and direction signals for the motor driver circuitry. 

## encoder.py 
Implements an interface for reading quadrature encoder signals from the wheel motors. It converts raw timer counts into position and velocity measurements used for motor control and state estimation. 

## task_line.py 
Implements the line-following behavior using data from the line sensor array. It calculates the line position error and adjusts the left and right wheel speeds using a PI control strategy to keep the robot centered on the line. 

## line_sensor.py 
Provides a driver for the QTR-8A reflective infrared sensor array used for line detection. It reads analog sensor values and computes a centroid that represents the position of the line relative to the robot. 

## task_recover.py 
Controls the robot’s higher-level navigation and recovery behaviors. It manages sequences such as line loss recovery, heading changes, bump sensor responses, and special navigation states. 

## heading_nav.py 
Implements heading-based navigation utilities using IMU data. It allows the robot to maintain or turn to specific headings while driving. 

## IMU_driver.py 
Low-level driver for communicating with the BNO055 inertial measurement unit. It reads orientation and angular velocity data used for navigation and heading control. 

## IMU_task.py 
A task responsible for continuously reading the IMU and updating shared variables with orientation and angular velocity measurements. 

## task_estimator.py 
Implements the robot state estimator that tracks position and orientation using encoder and IMU data. It provides estimated state variables used by navigation and control tasks. 

## Matrices.py 
Contains the state-space matrices used by the estimator model. These matrices define the mathematical relationship between system inputs, states, and outputs. 

## Matrix calculations.py 
A helper script used to compute or verify the matrices used by the estimator. This file is typically used offline for development and testing rather than running on the robot. 