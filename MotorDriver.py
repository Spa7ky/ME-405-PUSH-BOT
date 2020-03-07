'''MotorDriver.py
'''
import pyb
class MotorDriver:
    '''This class creates a motor driver for the ME 405 board. An object can be made with an enable pin, two motor pins, and a timer pin as an input.'''
    def __init__(self, enablePin1 = 'PA10',motorPin1 = 'PB4',motorPin2 = 'PB5',motortimer = 3):
        '''Initializes the motor. This defaults pins PB4 and PB5 as the motor control pins, pin PA10 as the motor enable pin, and Timer 3 as the motor timer. Any other valid pin-timer input will work as well.'''
        ## Takes the first input pin and sets it as an output. This will enable the motor for use.
        self.enablePin1 = pyb.Pin (enablePin1,pyb.Pin.OUT_PP)
        ## Takes the second input pin and sets it as an output for driving the motor.
        self.motorPin1 = pyb.Pin (motorPin1,pyb.Pin.OUT_PP)
        ## Takes the third input pin and sets it as an output for driving the motor.
        self.motorPin2 = pyb.Pin (motorPin2,pyb.Pin.OUT_PP)
        ## Enables the motor by setting the enable pin high.
        self.enablePin1.high()
        ## Creates timer object based on timer input and sets the frequency it runs at.
        self.motortimer = pyb.Timer(motortimer,freq=20000)
        ## Creates Ch1 for the Timer that has been input.
        self.motorCh1 = self.motortimer.channel (1,pyb.Timer.PWM,pin = self.motorPin1)
        ## Creates Ch2 for the Timer that has been input.
        self.motorCh2 = self.motortimer.channel (2,pyb.Timer.PWM,pin = self.motorPin2)
    def set_duty_cycle(self,level=0):
        '''Sends a PWM signal to the motor based on the variable 'level'. Level can be any number from -100 to 100 and will default to 0 for safety reasons. If level is a positve value then it will rotate the motor clockwise, and will rotate the motor counterclockwise for negative values. '''
        if level > 100:
            level = 100
        elif level < (-1)*(100):
            level = (-1)*(100)
        if level < 0:
            self.motorCh2.pulse_width_percent (abs(level))
            self.motorCh1.pulse_width_percent (0)
        else:
            self.motorCh1.pulse_width_percent (level)
            self.motorCh2.pulse_width_percent (0)
        return None