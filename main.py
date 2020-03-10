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
import print_task
#from necir import NecIr

# Allocate memory so that exceptions raised in interrupt service routines can
# generate useful diagnostic printouts
alloc_emergency_exception_buf (100)

def MotorControlTask():
    PosControl_flag = 0
    VelControl_flag = 0
    state = 0
    TOFMaxRange = 210
    Speed = 1.5
    ActivateKey = 74
    while True: 
        if IRShare.get() != ActivateKey:
            state = 0
            controllerR.changeKpVel(20)
            controllerL.changeKpVel(20)
            controllerR.changeVelSetpoint(0)
            controllerL.changeVelSetpoint(0)
        if PosControl_flag == 1:
            
        # Position Control Setup
        #Right Motor
            measured_location = encoderR.Position()
            level = controllerR.Pos_control(measured_location)
            motorR.set_duty_cycle(level)
        #Left Motor
            measured_location = encoderL.Position()
            level = controllerL.Pos_control(measured_location)
            motorL.set_duty_cycle(level)
        if VelControl_flag == 1: 
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
                VelControl_flag = 1
                controllerR.changeKpVel(100)
                controllerL.changeKpVel(10)
                controllerR.changeVelSetpoint(-Speed)
                controllerL.changeVelSetpoint(-Speed)
        #Right Scan State      
        elif state == 1:
            #print('state 1')
            if TOF1Share.get()< TOFMaxRange or TOF2Share.get()<TOFMaxRange:
                state = 3
                controllerR.changeVelSetpoint(Speed)
                controllerL.changeVelSetpoint(-Speed)
        #Left Scan State
        elif state == 2:
            #print('state 2')
            if TOF1Share.get()< TOFMaxRange or TOF2Share.get()<TOFMaxRange:
                state = 3
                controllerR.changeVelSetpoint(Speed)
                controllerL.changeVelSetpoint(-Speed)
        #Drive Straight State        
        elif state == 3:
            #print('state 3')
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
            if TOF1Share.get()>TOFMaxRange and TOF2Share.get()<TOFMaxRange:
                scan_flag = 'Left'
            if TOF1Share.get()<TOFMaxRange and TOF2Share.get()>TOFMaxRange:
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
        elif state == TURNING:
            ticksoldR = ticksnewR
            ticksoldL = ticksnewL
            ticksnewR = encoderR.Position()
            ticksnewL = encoderL.Position()
            deltaR = ticksnewR-ticksoldR
            #print('deltaR'+str(deltaR)+' ticks')
            deltaL = ticksnewL-ticksoldL
            #print('deltaL'+str(deltaL)+' ticks')
            danglec = deltaR*(rwheel/rbase)*((2*3.1415)/1440)
            totalAngle += danglec
            #print('Total Angle = '+str(totalAngle*(180/3.1415))+' degrees')
            #anglec = (180-(shareIMU.get()*sampleRate))*.01745329
            if deltaR>0 and deltaL>0 or deltaR<0 and deltaL<0:
                state = STRAIGHT
                anglec = 3.1415 - totalAngle
                ticksnewR = encoderR.Position()
                ticksnewL = encoderL.Position()
        yield state
'''
def IR_isr(IRtimer):        # timer capture value is passed into callback
    global fullFlag, start, pauseFlag
    if fullFlag == 0: # and pauseFlag == 0:
        q0.put(IRtimer.channel(1).capture())
        #start = utime.ticks_ms()
        #print(start)
    if q0.full():
        fullFlag = 1
def ParsingTask():
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
    print('entered Parsing Task')
    while True:
        if utime.ticks_diff(utime.ticks_ms(),start)>20 and utime.ticks_diff(utime.ticks_ms(),start)<130 and q0.any():
            pauseFlag = 1 
            print(utime.ticks_diff(utime.ticks_ms(),start))
            while q0.any():
                trash.append(q0.get())
                print('trashcan'+str(trash[-1]))
            del trash[:]
            print('queue cleared')
            pauseFlag = 0
            utime.sleep_ms(5)
            
        print('in while loop')
        try:
            print('tried')
            if fullFlag == 1:
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
                    list1 = parseData[:8]
                    list2 = parseData[8:16]
                    list3 = parseData[16:24]
                    list4 = parseData[24:]
                    listToStr = ''.join([str(elem) for elem in parseData]) 
                    listToStr1 = ''.join([str(elem) for elem in list1]) 
                    listToStr2 = ''.join([str(elem) for elem in list2])
                    listToStr3 = ''.join([str(elem) for elem in list3]) 
                    listToStr4 = ''.join([str(elem) for elem in list4])
                    Address = (int(listToStr1,2))
                    Command = (int(listToStr3,2))
                    IRShare.put(int(Command))
                    #print('IRShare Command '+str(IRShare.get()))
                    #print('----------New Packet----------\nRaw:      0b'+str(listToStr)+'\n\n ADDR:    0b'+str(listToStr1)+'\nnADDR:    0b'+str(listToStr2)+'\n  CMD:    0b'+str(listToStr3)+'\n nCMD:    0b'+str(listToStr4)+'\n\nAddress (Decimal):    '+str(Address)+'\nCommand (Decimal):    '+str(Command)+'\n\n')
                    del IRdata[:]
                    del pulseWidth[:]
                    del parseData[:]
                    fullFlag = 0
        except StopIteration:
            print('Nah Homie')
            break
    yield None
        
            
#def nec_cb(nec, a, c, r):
    #IRShare.put(c)
    #print('IR Share Online '+str(IRShare.get()))
'''
# This code is from team MECHA 12 Author Schuyler Ryan
def IR_command_sensor():
    #Empty the Queue
    InterruptQueue._wr_idx = 0
    InterruptQueue._rd_idx = 0
    InterruptQueue._num_items = 0
    # DataPacket is the full list of Timer 1 counts.
    DataPacket = []
    # TimeDiffs is the list of differences in time from DataPacket 
    TimeDiffs = []
    # LogicList is the list of binary data converted from TimeDiffs
    LogicList = []
    # RepeatFlag is the flag that tells if a repeat code has been recieved
    RepeatFlag = False
    # Found is the flag that tells if a full complete Data Packet is found
    Found = False
    # ShutDown is the flag that tells the motors to stop working
    # We are starting the bot in the shutdown position
    #ShutDownFlag.put(True)
    while True:
        #print('IR')
        if InterruptQueue._num_items >= 68 or InterruptQueue.full():
            while not InterruptQueue.empty(): 
                if InterruptQueue._num_items == 1:
                    #If queue has only 1 term left, just store the Time Difference
                    DataPacket[-1] += InterruptQueue.get()
                    #The queue should now be empty
                else: 
                    #Store a term from the queue
                    DataPacket.append(-InterruptQueue.get())
                    #If there are atleast 2 terms already:
                    if len(DataPacket) > 1:
                        #Calculate the Time Difference between terms
                        DataPacket[-2] -= DataPacket[-1]
                        while DataPacket[-2] < 0:
                            #Correct for overflow
                            DataPacket[-2] += 65535
                if len(DataPacket) > 1 and DataPacket[-1] > 1850 and DataPacket[-1] < 3000 and DataPacket[-2] > 8000 and DataPacket[-2] < 10000 and RepeatFlag == False:
                    #If a repeat code is read:
                    DataPacket = DataPacket[:-2] #Ignore the last 2 Time Differences
                    RepeatFlag = True #Repeat code recognized
                    #print_task.put('Repeat Code Found\n')
            #Turn interrupts off while processing data
            irq_state = pyb.disable_irq ()
            #Found is the flag that indicates that an entire valid data packet is found
            TimeDiffs = DataPacket
            for i in range(len(TimeDiffs)):
                #For every time difference:
                if TimeDiffs[i] >= 3800 and TimeDiffs[i] <= 5200:
                    #If an initial pulse is found:
                    try:
                        TimeDiffs[i+64] #Test if enough data exists
                        Found = True #Raise the valid data packet flag
                        TimeDiffs = TimeDiffs[i+1:i+65] #Ignore everything except the current data packet
                        #0->64 represents data package
                        #65->66 represents trailing pulse and wait time
                        break
                    except:
                        for x in DataPacket:
                            #Put the data packet back into the queue
                            InterruptQueue.put(x)
            if Found:
                #ignore down edges for reading binary
                TimeDiffs = [TimeDiffs[x] for x in range(len(TimeDiffs)) if x%2]
                #print(TimeDiffs)
                for time in TimeDiffs:
                    if time > 450 and time < 700:
                        LogicList.append(0)
                    elif time > 1500 and time < 1800:
                        LogicList.append(1)
                    else:
                        LogicList.append('ERROR')
                if not 'ERROR' in LogicList:
                    #RAW = (''.join(map(str,LogicList)))
                    #ADDR = (''.join(map(str,LogicList[0:8])))
                    #nADDR = (''.join(map(str,LogicList[8:16])))
                    CMD = int(''.join(map(str,LogicList[16:24])), 2)
                    IRShare.put(CMD)
                    print('IR share'+str(IRShare.get()))
                    nCMD = int(''.join(map(str,LogicList[24:32])), 2)
                    #If the start signal is recieved:
                    if (CMD^nCMD) == 255 and CMD == 74:
                        ShutDownFlag.put(False)
                        print_task.put_bytes(str.encode('\n'+str(CMD)+'\n'))
                    elif (CMD^nCMD) == 255 and CMD != 74:
                        ShutDownFlag.put(True)
                        print_task.put_bytes(str.encode('\n'+str(CMD)+'\n'))
                    else:
                        pass
                else:
                    pass
            #Clear all flags and lists    
            Found = False        
            DataPacket = []         
            TimeDiffs = []
            LogicList = []
            RepeatFlag = False
            #ShutDownFlag.put(False)
            #Empty the Queue
            InterruptQueue._wr_idx = 0
            InterruptQueue._rd_idx = 0
            InterruptQueue._num_items = 0
            #Turn interrupts back on
            irq_state = pyb.enable_irq (irq_state)
            InterruptQueue.get()
        yield(0)
def irInterrupt(time1):
    if not InterruptQueue.full():
        InterruptQueue.put(time1.counter(), in_ISR=True)
        
irq_state = pyb.disable_irq()
time1 = pyb.Timer(1, prescaler=79, period = 65535)      
pinA8 = pyb.Pin(pyb.Pin.board.PA8, pyb.Pin.IN)       
time1.channel(1, pyb.Timer.IC, pin=pinA8, polarity=pyb.Timer.BOTH, callback=irInterrupt)
    
ShutDownFlag = task_share.Share ('h', thread_protect = True, name = "ShutDown")
ShutDownFlag.put(True)
InterruptQueue = task_share.Queue ('H', 200, thread_protect = False, 
                                   overwrite = False, name = "Interrupt_Queue")

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
#nec = NecIr()
#nec.callback(nec_cb) 


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
    task6 = cotask.Task (IR_command_sensor, name = 'Task6 IR' ,priority = 4,
                        period = 1000, profile = True, trace = False)
    #task7 = cotask.Task (ParsingTask, name = 'Task7 IR' ,priority = 4,
    #                    period = 1, profile = True, trace = False)
    #cotask.task_list.append (task1)
    #cotask.task_list.append (task2)
    #cotask.task_list.append (task3)
    #cotask.task_list.append (task4)
    #cotask.task_list.append (task5)
    cotask.task_list.append (task6)
    #cotask.task_list.append (task7)
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