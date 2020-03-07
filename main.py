import pyb
from micropython import const, alloc_emergency_exception_buf
import gc

import cotask
import task_share
from Encoder import EncoderDriver
from Controller import ClosedLoopDriver
from MotorDriver import MotorDriver
from VL53L0X import VL53L0X
from machine import I2C
import utime
from imu import MPU6050

# Allocate memory so that exceptions raised in interrupt service routines can
# generate useful diagnostic printouts
alloc_emergency_exception_buf (100)

def Turn(direction = 1,angle = 60): # 0 = counter-clockwise, 1 = clockwise, angle in degrees
    ticks = angle*4
    if direction == 0:
        controllerR.setpoint(ticks)
        controllerL.setpoint(-ticks)
    if direction == 1:
        controllerR.setpoint(-ticks)
        controllerL.setpoint(ticks)
def MotorControlTask():
    encoderR = EncoderDriver('PB6','PB7',4)
    controllerR = ClosedLoopDriver(-200000,.04)
    motorR = MotorDriver('PB10','PB4','PB5',3)
    encoderL = EncoderDriver('PC6','PC7',8)
    controllerL = ClosedLoopDriver(-100000,.04)
    motorL = MotorDriver('PC1','PA0','PA1',5)    
    while True: 
        # Position Control Setup
        measured_location = encoderR.EncoderTask()
        level = controllerR.closed_loop(measured_location)
        motorR.set_duty_cycle(level)    
        yield None
def TOF_Task():
    TOF1 = VL53L0X(i2c)
    TOF2 = VL53L0X(i2c2)
    TOF1.start()
    TOF2.start()
    TOF1Queue = task_share.Share('l')
    TOF2Queue = task_share.Share('l')
    while True:
        TOF1Queue.put(TOF1.read())
        TOF2Queue.put(TOF2.read())
        print(TOF1Queue.get())
        print(TOF2Queue.get())
        yield None
def IMU_Task():
#i2c = I2C(1,freq = 200000)
    shareIMU = task_share.Share('f')
    IMU = MPU6050(i2c)
    while True:
        shareIMU.put(IMU.accel.x)
        print(shareIMU.get())
        yield None
def LineSensorTask():
    shareLine = task_share.Share('i')
    PA9 = pyb.Pin ('PA9',pyb.Pin.IN)
    while True:
        shareLine.put(PA9.value())
        print(shareLine.get())
        yield None
def PositionTrackingTask():
    motorR.Position
i2c = I2C(1,freq = 200000) #Uses bus 3 because of the pins it is connected to
i2c2 = I2C(3,freq = 200000)
if __name__ == "__main__":

    print ('\033[2JTesting scheduler in cotask.py\n')
    #task1 = cotask.Task (MoEnTask1, name = 'Task1' ,priority = 3,
    #                    period = 50, profile = True, trace = False)
    task2 = cotask.Task (LineSensorTask, name = 'Task2' ,priority = 3,
                        period = 500, profile = True, trace = False)
    task3 = cotask.Task (IMU_Task, name = 'Task3' ,priority = 3,
                        period = 500, profile = True, trace = False)
    task4 = cotask.Task (TOF_Task, name = 'Task4' ,priority = 3,
                        period = 500, profile = True, trace = False)
    #cotask.task_list.append (task1)
    cotask.task_list.append (task2)
    cotask.task_list.append (task3)
    cotask.task_list.append (task4)
    # A task which prints characters from a queue has automatically been
    # created in print_task.py; it is accessed by print_task.put_bytes()


    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect ()

    # Run the scheduler with the chosen scheduling algorithm. Quit if any 
    # character is sent through the serial port
    vcp = pyb.USB_VCP ()
    while not vcp.any ():
        cotask.task_list.pri_sched ()

    # Empty the comm port buffer of the character(s) just pressed
    vcp.read ()

    # Print a table of task data and a table of shared information data
    print ('\n' + str (cotask.task_list) + '\n')
    print (task_share.show_all ())
    #print (task1.get_trace ())
    print (task2.get_trace ())
    print (task3.get_trace ())
    print (task4.get_trace ())
    print ('\r\n')