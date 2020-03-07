'''@file Controller.py
'''
import utime

class ClosedLoopDriver:
    '''This class creates a closed loop motor driver for the ME 405 board. It takes a setpoint and a Kp value as an input.'''
    def __init__(self,Pos_setpoint = 20000,Vel_setpoint = 1.5,Kp = 1):
        '''Initializes the motor controller object. This defualts to a setpoint of 20000 and a Kp value of 1. These can be changed when generating an object.'''
        ## Defines the setpoint
        self.Pos_setpoint = Pos_setpoint
        self.Vel_setpoint = Vel_setpoint
        ## Defines the Kp value
        self.Kp = Kp
        ## Creates an empty list for the variable time.
        self.time = []
        ## Creates an empty list for the variable position.
        self.position = []
        ## Creates an empty list for the variable response.
        self.response = []
        self.deltaT = .05
    def Pos_control(self,measured_location):
        '''Does the calculations required for a proportional closed loop controller. The error is found by subtracting the current location to the setpoint. The actuation signal is determined by multiplying the error by the Kp value.'''
        ## Error is the difference between the setpoint and the current location. 
        error = self.Pos_setpoint -measured_location
        ## Actuation signal is the PWM signal to be sent to the motor based on the error.
        actuation_sig = error * self.Kp
        ## Takes the current time and stores it in a growing list.
        return actuation_sig
    def Vel_control(self,deltaP):
        '''Does the calculations required for a proportional closed loop controller. The error is found by subtracting the current location to the setpoint. The actuation signal is determined by multiplying the error by the Kp value.'''
        current_velocity = (deltaP/self.deltaT)*(1/1440)*(2*3.14)*(1/12)
        ## Error is the difference between the setpoint and the current location. 
        error = self.Vel_setpoint -current_velocity
        ## Actuation signal is the PWM signal to be sent to the motor based on the error.
        actuation_sig = error * self.Kp
        #print(current_velocity)
        return actuation_sig
    def changeSetpoint(self,setpoint):
        '''Allows the user to change the setpoint of the motor.'''
        ## Defines setpoint as the integer that was input into the function.
        self.setpoint = setpoint
    def changeKp(self,Kp):
        '''Allows user to change the Kp value.'''
        ## Defines Kp as the integer that was input into the function.
        self.Kp = Kp
    def printStep(self):
        '''This function prints the step response of the motor. It will print the values in csv format with time in the first column and position in the second.'''
        ## turns time list into a list of strings.
        self.time = [str(i) for i in self.time]
        ## turns position list into a list of strings.
        self.position = [str(i) for i in self.position]
        for i in range(len(self.time)):
            ## Joins them together as a comma separated value in response list.
            self.response.append(','.join([self.time[i],self.position[i]]))
        for i in range(len(self.time)):    
            print(self.response[i])