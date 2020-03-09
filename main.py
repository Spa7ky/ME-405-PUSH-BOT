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
import math

# Allocate memory so that exceptions raised in interrupt service routines can
# generate useful diagnostic printouts
alloc_emergency_exception_buf (100)

def MotorControlTask():
    #STOPPED = const(0)
    #SCANRIGHT = const(1)
    #SCANLEFT = const(2)
    #FORWARD = const(3)
    state = 1
    controllerR = ClosedLoopDriver(200000,1,10000)
    motorR = MotorDriver()
    controllerL = ClosedLoopDriver(200000,1,10000)
    motorL = MotorDriver('PC1','PA0','PA1',5, 100)
    while True: 
        '''
        # Position Control Setup
        #Right Motor
        measured_location = encoderR.Position()
        level = controllerR.Pos_control(measured_location)
        motorR.set_duty_cycle(level)
        #Left Motor
        measured_location = encoderL.Position()
        level = controllerL.Pos_control(measured_location)
        motorL.set_duty_cycle(level)
        '''
        # Velocity Control Setup
        #Right Motor
        measured_velocity = encoderR.Velocity()
        level = controllerR.Vel_control(measured_velocity)
        motorR.set_duty_cycle(level) 
        #Left Motor
        measured_velocity = encoderL.Velocity()
        level = controllerL.Vel_control(measured_velocity)
        motorL.set_duty_cycle(level) 
        
        print('Im in the control loop!')
        if state == const(0):  #STOP State
            print('state 0')
            controllerR.changeVelSetpoint(0)
            controllerL.changeVelSetpoint(0)
            #if IRShare.get() == 1: #change tasks when receiving correct IR data
            #   state = 1
            
        #Right Scan State      
        elif state == const(1):
            print('state 1')
            controllerR.changeVelSetpoint(-1)
            controllerL.changeVelSetpoint(1)
            if TOF1Share.get()< 8000 or TOF2Share.get()<8000:
                state = 3
        #Left Scan State
        elif state == const(2):
            print('state 2')
            controllerR.changeVelSetpoint(1)
            controllerL.changeVelSetpoint(-1)
            if TOF1Share.get()< 8000 or TOF2Share.get()<8000:
                state = 3
        #Drive Straight State        
        elif state == const(3):
            print('state 3')
            controllerR.changeVelSetpoint(1.50)
            controllerL.changeVelSetpoint(1.50)
            #if shareLine.get() == 0:
            #    state = 4
            if TOF1Share.get()>8000 and TOF2Share.get()<8000:
                scan_flag = 'Left'
            if TOF1Share.get()<8000 and TOF2Share.get()>8000:
                scan_flag = 'Right'
            if TOF1Share.get()>8000 and TOF2Share.get()>8000:
                if scan_flag == 'Right':
                    state = 1
                elif scan_flag == 'Left':
                    state = 2
        elif state == const(4):
            print('state 4')
            state = 3
        yield state
        
def TOF_Task():
    i2c2 = I2C(3,freq = 200000) 
    TOF1 = VL53L0X(i2c)
    TOF2 = VL53L0X(i2c2)
    TOF1.start()
    TOF2.start()
    while True:
        TOF1Share.put(TOF1.read())
        TOF2Share.put(TOF2.read())
        print('TOFL '+str(TOF1Share.get())+' mm')
        print('TOFR '+str(TOF2Share.get())+' mm')
        yield None
def IMU_Task():
    IMU = MPU6050(i2c)
    while True:
        shareIMU.put(IMU.gyro.z)
        #print(shareIMU.get())
        yield None
def LineSensorTask():
    PA9 = pyb.Pin ('PA9',pyb.Pin.IN)
    while True:
        shareLine.put(PA9.value())
        #print(shareLine.get())
        yield None
def PositionTrackingTask(startOffset = 0,sampleRate = 1,rwheel = 1, rbase = 3.1875): # in, s
    A = startOffset#/.004363323 #ticks
    B = 0
    C = A
    ticksnewR = encoderR.Position()
    ticksnewL = encoderL.Position()
    anglec = 3.1415
    STRAIGHT = const (0)
    TURNING = const (1)
    state = 0
    totalAngle = 0
    while True:
        if state == STRAIGHT:
            ticksoldR = ticksnewR
            ticksoldL = ticksnewL
            ticksnewR = encoderR.Position()
            ticksnewL = encoderL.Position()
            deltaR = ticksnewR-ticksoldR
            print('deltaR'+str(deltaR))
            deltaL = ticksnewL-ticksoldL
            print('deltaL'+str(deltaL))
            A = C
            B = (ticksnewR-ticksoldR)*.004363323*rwheel
            C = math.sqrt(math.pow(A,2)+math.pow(B,2)-(2*A*B*math.cos(anglec)))
            print("C = "+str(C))
            print('anglec = '+str(anglec))
            if deltaR > 0 and deltaL < 0 or deltaR < 0 and deltaL >0:
                state = TURNING
        elif state == TURNING:
            ticksoldR = ticksnewR
            ticksoldL = ticksnewL
            ticksnewR = encoderR.Position()
            ticksnewL = encoderL.Position()
            deltaR = ticksnewR-ticksoldR
            print('deltaR'+str(deltaR)+' ticks')
            deltaL = ticksnewL-ticksoldL
            print('deltaL'+str(deltaL)+' ticks')
            danglec = deltaR*(rwheel/rbase)*((2*3.1415)/1440)
            totalAngle += danglec
            print('Total Angle = '+str(totalAngle*(180/3.1415))+' degrees')
            #anglec = (180-(shareIMU.get()*sampleRate))*.01745329
            if deltaR>0 and deltaL>0 or deltaR<0 and deltaL<0:
                state = STRAIGHT
                anglec = 3.1415 - totalAngle
                ticksnewR = encoderR.Position()
                ticksnewL = encoderL.Position()
        yield state
            

i2c = I2C(1,freq = 200000) #Uses bus 3 because of the pins it is connected to
TOF1Share = task_share.Share('l')
TOF2Share = task_share.Share('l')
shareIMU = task_share.Share('f')
shareLine = task_share.Share('i')  
encoderR = EncoderDriver('PB6','PB7',4,direction='clockwise')   
encoderL = EncoderDriver('PC6','PC7',8,direction='counterclockwise')   
if __name__ == "__main__":

    print ('\033[2JTesting scheduler in cotask.py\n')
    task1 = cotask.Task (MotorControlTask, name = 'Task1' ,priority = 5,
                        period = 50, profile = True, trace = False)
    task2 = cotask.Task (LineSensorTask, name = 'Task2' ,priority = 5,
                        period = 50, profile = True, trace = False)
    task3 = cotask.Task (IMU_Task, name = 'Task3' ,priority = 5,
                        period = 50, profile = True, trace = False)
    task4 = cotask.Task (TOF_Task, name = 'Task4' ,priority = 5,
                        period = 50, profile = True, trace = False)
    task5 = cotask.Task (PositionTrackingTask, name = 'Task5' ,priority = 5,
                        period = 50, profile = True, trace = False)
    cotask.task_list.append (task1)
    cotask.task_list.append (task2)
    cotask.task_list.append (task3)
    cotask.task_list.append (task4)
    cotask.task_list.append (task5)
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
        #print(task1.get_trace())

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