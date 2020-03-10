# @main.py
import pyb
import task_share
from micropython import alloc_emergency_exception_buf

#q0 = task_share.Queue ('I', 68, thread_protect = False, overwrite = False,
#                       name = "Queue_0")
#IRShare = task_share.Share('i')
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
    global fullFlag
    fullFlag = 0
    IRtimerCh1.callback(IR_isr)
    IRdata = []
    pulseWidth = []
    parseData = []
    while True:
        '''
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
            '''
        if fullFlag == 1:
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
                print('IRShare Command '+str(IRShare.get()))
                #print('----------New Packet----------\nRaw:      0b'+str(listToStr)+'\n\n ADDR:    0b'+str(listToStr1)+'\nnADDR:    0b'+str(listToStr2)+'\n  CMD:    0b'+str(listToStr3)+'\n nCMD:    0b'+str(listToStr4)+'\n\nAddress (Decimal):    '+str(Address)+'\nCommand (Decimal):    '+str(Command)+'\n\n')
                del IRdata[:]
                del pulseWidth[:]
                del parseData[:]
                fullFlag = 0
        yield None