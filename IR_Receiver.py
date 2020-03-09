# @main.py
import pyb
import task_share
from micropython import const, alloc_emergency_exception_buf
import gc
import cotask

alloc_emergency_exception_buf(100)
IRpin = pyb.Pin ('PA8',pyb.Pin.IN)
IRtimer = pyb.Timer(1,prescaler = 79,period = 65535)
IRtimerCh1 = IRtimer.channel (1,pyb.Timer.IC,pin = IRpin,polarity = IRtimer.BOTH)
collectFlag = 0
def IR_isr(IRtimer):        # timer capture value is passed into callback
    global collectFlag
    if collectFlag == 0: 
        q0.put(IRtimer.channel(1).capture())
    if collectFlag == 1:
        pass

def Parsing():
    global collectFlag
    INIT = const(0)
    INIT2 = const(1)
    DETECTION = const(2)
    WAITING = const(3)
    PARSING = const(4)
    state = 0
    edge = []
    leading = []
    IRdata = []
    pulseWidth = []
    parseData = []
    tolerance = 500
    toleranceLong = 1125
    #repeatCode = [9000,2250,563]
    leadingPulseRange = [9000,4500]
    highPulseRange = [1687]
    lowPulseRange = [562]
    while True: 
        if state == INIT:
            if q0.num_in() >= 3:
                for i in range(3):
                    edge.append(q0.get())
                state = 1
        elif state == INIT2:
            leading.append(edge[1]-edge[0])
            leading.append(edge[2]-edge[1])
            if leading[0] < leadingPulseRange[0]+toleranceLong and leading[0]> leadingPulseRange[0]-toleranceLong and leading[1] < leadingPulseRange[1]+toleranceLong and leading[1]> leadingPulseRange[1]-toleranceLong:
                state = 3
            else:
                state = 2
                del edge[:1]
                del leading[0]
        elif state == DETECTION:
            if q0.any():
                edge.append(q0.get())
                leading.append(edge[1]-edge[0])
                if leading[0] < leadingPulseRange[0]+toleranceLong and leading[0]> leadingPulseRange[0]-toleranceLong and leading[1] < leadingPulseRange[1]+toleranceLong and leading[1]> leadingPulseRange[1]-toleranceLong:
                    state = 3
                else:
                    print("discarded leading"+str(leading[0]))
                    del edge[0]
                    del leading[0]
        elif state == WAITING:
            for i in range(64):
                IRdata.append(q0.get())
            print("IRdata"+str(IRdata))
            print("length "+str(len(IRdata)))
            collectFlag = 1
            state = 4
        elif state == PARSING:
            for i in range(0,63,2):
                pulseWidth.append(IRdata[i+1]-IRdata[i])
                if pulseWidth[i//2] < 0:
                    pulseWidth[i//2]+=65536
            print("pulse Width "+str(pulseWidth))
            print("length"+str(len(pulseWidth)))
            for i in range(0,len(pulseWidth)-1,1):
                if pulseWidth[i]<highPulseRange[0]+tolerance and pulseWidth[i]>highPulseRange[0]-tolerance:
                    parseData.append(1)
                elif pulseWidth[i]<lowPulseRange[0]+tolerance and pulseWidth[i]>lowPulseRange[0]-tolerance:
                    parseData.append(0)
            list1 = parseData[:8]
            list2 = parseData[8:16]
            list3 = parseData[16:24]
            list4 = parseData[24:]
            listToStr = ''.join([str(elem) for elem in parseData]) 
            listToStr1 = ''.join([str(elem) for elem in list1]) 
            listToStr2 = ''.join([str(elem) for elem in list2])
            listToStr3 = ''.join([str(elem) for elem in list3]) 
            listToStr4 = ''.join([str(elem) for elem in list4])
            print(listToStr)
            print(listToStr1)
            print(listToStr2)
            print(listToStr3)
            print(listToStr4)
            Address = (int(listToStr1,2))
            Command = (int(listToStr3,2))
            print('----------New Packet----------\nRaw:      0b'+str(listToStr)+'\n\n ADDR:    0b'+str(listToStr1)+'\nnADDR:    0b'+str(listToStr2)+'\n  CMD:    0b'+str(listToStr3)+'\n nCMD:    0b'+str(listToStr4)+'\n\nAddress (Decimal):    '+str(Address)+'\nCommand (Decimal):    '+str(Command)+'\n\n')
            del IRdata[:]
            del pulseWidth[:]
            del parseData[:]
            collectFlag = 0
            state = 0
        yield state
q0 = task_share.Queue ('I', 68, thread_protect = False, overwrite = False,
                           name = "Queue_0")        
IRtimerCh1.callback(IR_isr)
print ('\033[2JTesting scheduler in cotask.py\n')
task1 = cotask.Task (Parsing, name = 'Task1' ,priority = 3,
                period = 1, profile = True, trace = False)
cotask.task_list.append (task1)
gc.collect ()

while True:
    cotask.task_list.pri_sched ()

print ('\n' + str (cotask.task_list) + '\n')
print (task_share.show_all ())
print (task1.get_trace ())
print ('\r\n')

