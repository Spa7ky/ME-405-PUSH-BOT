# -*- coding: utf-8 -*-
"""
Created on Sat Mar  7 18:25:48 2020

@author: gwalt
"""
def MotorControlTask():
    state = 0
    controllerR = ClosedLoopDriver(0,0,.4)
    motorR = MotorDriver('PB10','PB4','PB5',3)
    encoderL = EncoderDriver('PC6','PC7',8)
    controllerL = ClosedLoopDriver(0,0,10000)
    motorL = MotorDriver('PC1','PA0','PA1',5)  
    while True: 
        # Position Control Setup
        #Right Motor
        measured_location = encoderR.Position()
        level = controllerR.Pos_control(measured_location)
        motorR.set_duty_cycle(level)
        #Left Motor
        measured_location = encoderL.Position()
        level = controllerL.Pos_control(measured_location)
        motorL.set_duty_cycle(level)
        # Velocity Control Setup
        #Right Motor
        measured_velocity = encoderR.Velocity()
        level = controllerR.Vel_control(measured_velocity)
        motorR.set_duty_cycle(level) 
        #Left Motor
        measured_velocity = encoderL.Velocity()
        level = controllerL.Vel_control(measured_velocity)
        motorL.set_duty_cycle(level) 
        if state == 0:  #STOP State
            controllerR.changeVelSetpoint(0)
            controllerL.changeVelSetpoint(0)
            if IRShare.get() == 1: #change tasks when receiving correct IR data
                state = 1
            
        #Right Scan State      
        if state == 1:
            controllerR.changeVelSetpoint(-.25)
            controllerL.changeVelSetpoint(.25)
            if TOF1Share.get()< 8000 or TOF2Share.get()<8000:
                state = 3
        #Left Scan State
        elif state == 2:
            controllerR.changeVelSetpoint(.25)
            controllerL.changeVelSetpoint(-.25)
            if TOF1Share.get()< 8000 or TOF2Share.get()<8000:
                state = 3
        #Drive Straight State        
        elif state == 3:
            controllerR.changeVelSetpoint(1.50)
            controllerL.changeVelSetpoint(1.50)
            if ShareLine.get() == 1:
                state = 4
            if TOF1Share.get>8000 and TOF2Share.get()<8000:
                scan_flag = 'Left'
            if TOF1Share.get()<8000 and TOF2Share.get()>8000:
                scan_flag = 'Right'
            if TOF1Share.get()>8000 and TOF2Share.get()>8000:
                if scan_flag == 'Right':
                    state = 1
                elif scan_flag == 'Left':
                    state = 2
                
        yield state