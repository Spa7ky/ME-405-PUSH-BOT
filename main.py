''' @file main.py
    @author Aaron Erickson and Garrison Walters
'''
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
#from necir import NecIr

# Allocate memory so that exceptions raised in interrupt service routines can
# generate useful diagnostic printouts
alloc_emergency_exception_buf (100)

def MotorControlTask():
    ''' The Motor control task controls the two motors on the robot simultaneously 
    in a velocity and position feedback loop. The loop and calculations are run simultaneously
    but flags control which control lopp is used to set PWM level. '''
    PosControl_flag = 0     # Allows closed loop control to be utilized 
    VelControl_flag = 0     # Allows velocity closed loop control to be used
    state = 0               # Sets initialization state
    TOFMaxRange = 400       # Sets scan range of TOF sensor
    Speed = 1.5             # Speed setpoint in ft-s
    ActivateKey = 48        # IR activation key (1) press 
    scan_flag = 'Right'     # Initialization of scan direction
    VelControl_flag = 1     # Begins closed loop control using velocity
    controllerR.changeKpVel(10000) # Sets Kv value
    controllerL.changeKpVel(10000)
                
    controllerR.changeVelSetpoint(-Speed) # Sets starting speed
    controllerL.changeVelSetpoint(-Speed)
    while True: 
        if IRShare.get() != ActivateKey:    # State 0, does not exit until IR command is recieved
            state = 0 
            controllerR.changeKpVel(20)     # changes Kv value for stopping
            controllerL.changeKpVel(20)
            controllerR.changeVelSetpoint(0)#sets velocity to 0
            controllerL.changeVelSetpoint(0)
        if PosControl_flag == 1:            # allows position control to be used for following states
            
        # Position Control Setup
        #Right Motor
            measured_location = encoderR.Position()
            level = controllerR.Pos_control(measured_location)
            motorR.set_duty_cycle(level)
        #Left Motor
            measured_location = encoderL.Position()
            level = controllerL.Pos_control(measured_location)
            motorL.set_duty_cycle(level)
        if VelControl_flag == 1:            # allows velocity control to be used for following states
        # Velocity Control Setup
        #Right Motor
            measured_velocity = encoderR.Velocity()
            level = controllerR.Vel_control(measured_velocity)
            motorR.set_duty_cycle(level) 
        #Left Motor
            measured_velocity = encoderL.Velocity()
            level = controllerL.Vel_control(measured_velocity)
            motorL.set_duty_cycle(level) 
        
        #print('Im in the control loop!')
        if state == 0 :  #STOP State
            #prin0t('state 0')
            #print('Print IR Share main'+str(IRShare.get()))
            if IRShare.get() == ActivateKey: #change tasks when receiving correct IR data
                state = 1
                print('state1')
        #Right Scan State      
        elif state == 1:
            print('state 1')
            if TOF1Share.get()< TOFMaxRange or TOF2Share.get()<TOFMaxRange:
                state = 3
                controllerR.changeVelSetpoint(Speed)
                controllerL.changeVelSetpoint(-Speed)
        #Left Scan State
        elif state == 2:
            print('state 2')
            if TOF1Share.get()< TOFMaxRange or TOF2Share.get()<TOFMaxRange:
                state = 3
                controllerR.changeVelSetpoint(Speed)
                controllerL.changeVelSetpoint(-Speed)
        #Drive Straight State        
        elif state == 3:
            print('state 3')
            #print('Encoder R '+str(encoderR.Velocity()))
            #print('Encoder L '+str(encoderL.Velocity()))
            if shareLine.get() == 0:
                state = 4
                VelControl_flag = 0
                PosControl_flag = 1
                deltaR = encoderR.Position() - 687
                deltaL = encoderL.Position() - 687
                controllerR.changePosSetpoint(deltaR)
                controllerL.changePosSetpoint(deltaL)
            if TOF1Share.get()>TOFMaxRange and TOF2Share.get()<TOFMaxRange: # tracks if bot traveled from right to left
                scan_flag = 'Left'
            if TOF1Share.get()<TOFMaxRange and TOF2Share.get()>TOFMaxRange: # left to rigt
                scan_flag = 'Right'
            if TOF1Share.get()>TOFMaxRange and TOF2Share.get()>TOFMaxRange:
                if scan_flag == 'Right':
                    state = 1
                    controllerR.changeVelSetpoint(-Speed)
                    controllerL.changeVelSetpoint(-Speed)
                elif scan_flag == 'Left':
                    state = 2
                    controllerR.changeVelSetpoint(Speed)
                    controllerL.changeVelSetpoint(Speed)
        # line sensor task
        elif state == 4:
            #print('state 4')
            if encoderR.Position() == deltaR and encoderL.Position() == deltaL:
                state = 1
                VelControl_flag = 1
                PosControl_flag = 0
                controllerR.changeVelSetpoint(-Speed)
                controllerL.changeVelSetpoint(Speed)
        yield state
def TOF_Task():
    ''' Time of flight sensor task. Sensors are initialized on an i2c bus and data
    is collected based on the period of the task data is collected in a share'''
    i2c2 = I2C(3,freq = 200000) 
    TOF1 = VL53L0X(i2c)
    TOF2 = VL53L0X(i2c2)
    TOF1.start()
    TOF2.start()
    while True:
        TOF1Share.put(TOF1.read())
        TOF2Share.put(TOF2.read())
        #print('TOFL '+str(TOF1Share.get())+' mm')
        #print('TOFR '+str(TOF2Share.get())+' mm')
        yield None
def IMU_Task():
    ''' Accelerometer is initialized on and i2c bus and data is collected with a share'''
    IMU = MPU6050(i2c)
    while True:
        shareIMU.put(IMU.gyro.z)
        #print(shareIMU.get())
        yield None
def LineSensorTask():
    ''' Infared line sensor's digital out put is collected here and stored in a share'''
    PA9 = pyb.Pin ('PA9',pyb.Pin.IN)
    while True:
        shareLine.put(PA9.value())
        #print(shareLine.get())
        yield None
def PositionTrackingTask(startOffset = 0,sampleRate = 1,rwheel = 1, rbase = 3.1875): # in, s
    ''' Position tracking through odemetry occurs here. Data from the encoders is collected 
    and data is interpereted to determine if a turn is being executed. If a turn is occuring 
    the angle of turn is calculated by using the known wheel base width and wheel radius
    If the bot is traveling strait after a turn the resultant vector is calculated using 
    law of cosines'''
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
        if state == STRAIGHT:       # state 0 strait operating behavior is tracked
            ticksoldR = ticksnewR
            ticksoldL = ticksnewL
            ticksnewR = encoderR.Position()
            ticksnewL = encoderL.Position()
            deltaR = ticksnewR-ticksoldR
            #print('deltaR'+str(deltaR))
            deltaL = ticksnewL-ticksoldL
            #print('deltaL'+str(deltaL))
            A = C
            B = (ticksnewR-ticksoldR)*.004363323*rwheel
            C = math.sqrt(math.pow(A,2)+math.pow(B,2)-(2*A*B*math.cos(anglec)))
            #print("C = "+str(C))
            #print('anglec = '+str(anglec))
            if deltaR > 0 and deltaL < 0 or deltaR < 0 and deltaL >0:
                state = TURNING
        elif state == TURNING:      # state 1 turning operating behavior is tracked
            ticksoldR = ticksnewR
            ticksoldL = ticksnewL
            ticksnewR = encoderR.Position()
            ticksnewL = encoderL.Position()
            deltaR = ticksnewR-ticksoldR
            #print('deltaR'+str(deltaR)+' ticks')
            deltaL = ticksnewL-ticksoldL
            #print('deltaL'+str(deltaL)+' ticks')
            danglec = deltaR*(rwheel/rbase)*((2*3.1415)/1440) #conversion from ticks to radians
            totalAngle += danglec
            #print('Total Angle = '+str(totalAngle*(180/3.1415))+' degrees')
            #anglec = (180-(shareIMU.get()*sampleRate))*.01745329
            if deltaR>0 and deltaL>0 or deltaR<0 and deltaL<0:
                state = STRAIGHT
                anglec = 3.1415 - totalAngle
                ticksnewR = encoderR.Position()
                ticksnewL = encoderL.Position()
        yield state

def IR_isr(IRtimer):        # timer capture value is passed into callback
    '''Interupt set up in capture mode, collecting time when ir sensor pulse is recieved
    activated on rising and falling edge of signal'''
    global fullFlag, start, pauseFlag
    if fullFlag == 0: # and pauseFlag == 0:
        q0.put(IRtimer.channel(1).capture())
        #start = utime.ticks_ms()
        #print(start)
    if q0.full():
        fullFlag = 1
def ParsingTask():
    ''' interperates ir data recieved from the isr outputs the command into a share'''
    alloc_emergency_exception_buf(100)
    IRpin = pyb.Pin ('PA8',pyb.Pin.IN)
    IRtimer = pyb.Timer(1,prescaler = 79,period = 65535)
    IRtimerCh1 = IRtimer.channel (1,pyb.Timer.IC,pin = IRpin,polarity = IRtimer.BOTH)
    IRtimerCh1.callback(IR_isr)
    global fullFlag
    fullFlag = 0
    IRdata = []
    pulseWidth = []
    parseData = []
    #print('entered Parsing Task')
    while True:
        #print('tried')
        while fullFlag == 1:
            print('wasting time')
            IRdata.append(q0.get())
            if q0.empty():
                #print(IRdata)
                tolerance = 500
                toleranceLong = 1125
                repeatCode = [9000,2250,563]
                leadingPulseRange = [9000,4500]
                highPulseRange = [562,1687]
                lowPulseRange = [562,562]
                for i in range(len(IRdata)-1):
                    pulseWidth.append(IRdata[i+1]-IRdata[i])
                    if pulseWidth[i] < 0:
                        pulseWidth[i]+=65536
                for i in range(len(IRdata)-2):        
                    if pulseWidth[i]<repeatCode[0]+toleranceLong and pulseWidth[i]>repeatCode[0]-toleranceLong and pulseWidth[i+1]<repeatCode[1]+toleranceLong and pulseWidth[i+1]>repeatCode[1]-toleranceLong and pulseWidth[i+2]<repeatCode[2]+tolerance and pulseWidth[i+2]>repeatCode[2]-tolerance:
                        #print(pulseWidth)
                        del IRdata[:]
                        del pulseWidth[:]
                        del parseData[:]
                        return
                if pulseWidth[0]<leadingPulseRange[0]+toleranceLong and pulseWidth[0]>leadingPulseRange[0]-toleranceLong and pulseWidth[1]<leadingPulseRange[1]+toleranceLong and pulseWidth[1]>leadingPulseRange[1]-toleranceLong:
                    pulseWidth.remove(pulseWidth[0])
                    pulseWidth.remove(pulseWidth[0])
                else:
                    #print(pulseWidth)
                    del IRdata[:]
                    del pulseWidth[:]
                    del parseData[:]
                    return        
                for i in range(0,len(pulseWidth)-2,2):
                    if pulseWidth[i]<highPulseRange[0]+tolerance and pulseWidth[i]>highPulseRange[0]-tolerance and pulseWidth[i+1]<highPulseRange[1]+tolerance and pulseWidth[i+1]>highPulseRange[1]-tolerance:
                        parseData.append(1)
                    elif pulseWidth[i]<lowPulseRange[0]+tolerance and pulseWidth[i]>lowPulseRange[0]-tolerance and pulseWidth[i+1]<lowPulseRange[1]+tolerance and pulseWidth[i+1]>lowPulseRange[1]-tolerance:
                        parseData.append(0)
                #print(parseData)
                #print(pulseWidth)  
                #list1 = parseData[:8]
                #list2 = parseData[8:16]
                list3 = parseData[16:24]
                #list4 = parseData[24:]
                #listToStr = ''.join([str(elem) for elem in parseData]) 
                #listToStr1 = ''.join([str(elem) for elem in list1]) 
                #listToStr2 = ''.join([str(elem) for elem in list2])
                listToStr3 = ''.join([str(elem) for elem in list3]) 
                #listToStr4 = ''.join([str(elem) for elem in list4])
                #Address = (int(listToStr1,2))
                Command = (int(listToStr3,2))
                IRShare.put(int(Command))
                print('IRShare Command '+str(IRShare.get()))
                #print('----------New Packet----------\nRaw:      0b'+str(listToStr)+'\n\n ADDR:    0b'+str(listToStr1)+'\nnADDR:    0b'+str(listToStr2)+'\n  CMD:    0b'+str(listToStr3)+'\n nCMD:    0b'+str(listToStr4)+'\n\nAddress (Decimal):    '+str(Address)+'\nCommand (Decimal):    '+str(Command)+'\n\n')
                del IRdata[:]
                del pulseWidth[:]
                del parseData[:]
                fullFlag = 0
                yield None
        yield None

'''Initialization of i2c, shares, queues, and objects used across multiple tasks'''
i2c = I2C(1,freq = 200000) #Uses bus 3 because of the pins it is connected to
TOF1Share = task_share.Share('l')
TOF2Share = task_share.Share('l')
q0 = task_share.Queue ('I', 68, thread_protect = False, overwrite = False,
                       name = "Queue_0")
IRShare = task_share.Share('i')
shareIMU = task_share.Share('f')
shareLine = task_share.Share('i')  
encoderR = EncoderDriver('PB6','PB7',4,direction='clockwise')   
encoderL = EncoderDriver('PC6','PC7',8,direction='counterclockwise') 
controllerR = ClosedLoopDriver(0,0,.2,100)
motorR = MotorDriver()
controllerL = ClosedLoopDriver(0,0,.2,20)
motorL = MotorDriver('PC1','PA0','PA1',5, 100) 

if __name__ == "__main__":

    print ('\033[2JTesting scheduler in cotask.py\n')
    task1 = cotask.Task (MotorControlTask, name = 'Task1 MC' ,priority = 6,
                        period = 50, profile = True, trace = False)
    task2 = cotask.Task (LineSensorTask, name = 'Task2 LS' ,priority = 5,
                        period = 100, profile = True, trace = False)
    task3 = cotask.Task (IMU_Task, name = 'Task3 IMU' ,priority = 1,
                        period = 50, profile = True, trace = False)
    task4 = cotask.Task (TOF_Task, name = 'Task4 TOF' ,priority = 5,
                        period = 100, profile = True, trace = False)
    task5 = cotask.Task (PositionTrackingTask, name = 'Task5 PosTr' ,priority = 2,
                        period = 50, profile = True, trace = False)
    task6 = cotask.Task (ParsingTask, name = 'Task6 IR' ,priority = 4,
                        period = 100, profile = True, trace = False)
    cotask.task_list.append (task1)
    cotask.task_list.append (task2)
    cotask.task_list.append (task3)
    cotask.task_list.append (task4)
    cotask.task_list.append (task5)
    cotask.task_list.append (task6)
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