""" @file main.py
    This file contains a program that runs two tasks, which independantly control two different motors. 

    @author Aaron Erickson and Garrison Walters


"""

import pyb
from micropython import const, alloc_emergency_exception_buf
import gc
## Imports encoder file with the encoder driver class, the controller file with controller
# driver class, and the motordriver file with the motor driver class. 
import cotask
import task_share
from Encoder import EncoderDriver
from Controller import ClosedLoopDriver
from MotorDriver import MotorDriver

# Allocate memory so that exceptions raised in interrupt service routines can
# generate useful diagnostic printouts
alloc_emergency_exception_buf (100)

# Declare some constants to make state machine code a little more readable.
# This is optional; some programmers prefer to use numbers to identify tasks
GOING = const (0)
STOPPED = const (1)

def MoEnTask1(encoder='enc1',controller='controller1',motor='motor1'):
    '''Creates the first motor controller task. This is done by creating the objects needed to pass through the encoder, controller, and motor classes. This task runs those three programs to create a closed loop motor controller.'''
    ## Creates encoder object using pins PB6 and PB7 on timer 4
    encoder = EncoderDriver('PB6','PB7',4)
    ## Creates controller object with a setpoint of -200000 and a Kp value of .04
    controller = ClosedLoopDriver(1440,1.5,10000)
    ## Creates motor object using PB10 as the enable pin and pins PB4 and PB5 as motor pins on timer 3.
    #motor = MotorDriver('PB10','PB4','PB5',3)
    motor = MotorDriver()
    while True:  
        deltaP = encoder.Velocity()
        level = controller.Vel_control(deltaP)
        motor.set_duty_cycle(level)    
        yield None
def MoEnTask2(encoder='enc2',controller='controller2',motor='motor2'):
    '''Creates the second motor controller task. This is done by creating the objects need to pass through the encoder, controllerm and motor classes. This task runs those three programs to create a closed loop motor controller separate from the first one.'''
    ## Creates encoder object using pins PC6 and PC7 on timer 8
    encoder = EncoderDriver('PC6','PC7',8)
    ## Creates controller object with a setpoint of -100000 and a Kp value of .04
    controller = ClosedLoopDriver(1440,1.5,.2)
    ## Creates motor object using PC1 as the enable pin and pins PA0 and PA1 on timer 5.
    motor = MotorDriver('PC1','PA0','PA1',5)
    while True:  
        measured_location = encoder.Position()
        level = controller.Pos_control(measured_location)
        motor.set_duty_cycle(level)    
        yield None

if __name__ == "__main__":

    print ('\033[2JTesting scheduler in cotask.py\n')
    task1 = cotask.Task (MoEnTask1, name = 'Task1' ,priority = 3,
                        period = 50, profile = True, trace = False)
    task2 = cotask.Task (MoEnTask2, name = 'Task2' ,priority = 3,
                        period = 50, profile = True, trace = False)
    cotask.task_list.append (task1)
    cotask.task_list.append (task2)
    


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
    print (task1.get_trace ())
    print ('\r\n')