'''@file EncoderDriver.py
'''
import pyb
class EncoderDriver:
    '''This class creates an encoder object for the ME 405 board. The encoder object will keep track of the motor position by using the two channel optical encoder that is attached to the motor.'''
    def __init__(self,encPin1='PB6',encPin2='PB7',enctimer=4,direction = 'clockwise'):
        '''Initializes the encoder. This defaults pins B6 and B7 as the encoder pins on Timer 4, but any valid pin-timer input will be placed.'''
        ## Takes the first pin put into the function and sets it as an input pin.
        self.encPin1 = pyb.Pin(encPin1,pyb.Pin.ALT)
        ## Takes the second pin put into the function and sets it as an input pin.
        self.encPin2 = pyb.Pin(encPin2,pyb.Pin.ALT)
        ## Creates a timer object based on the Timer input.
        self.enctimer = pyb.Timer(enctimer,prescaler=0,period=65535)
        ## Creates Ch1 for the Timer that has been input.
        self.encTimerCh1 = self.enctimer.channel(1,pyb.Timer.ENC_AB,pin=self.encPin1)
        ## Creates Ch2 for the Timer that has been input.
        self.encTimerCh2 = self.enctimer.channel(2,pyb.Timer.ENC_AB,pin=self.encPin2)
        ## Defines the new encoder position and sets it to zero.
        self.encNew = 0
        ## Defines the total encoder position and sets it to zero. 
        self.encTotal = 0
        self.direction = direction
    def read(self,name='motor'):
        '''Prints the current position of the motor'''
        print('Current position of '+str(name)+' is '+str(self.encTotal))
    def Position(self):
        if self.direction == 'clockwise':
            '''Calculates the current position of the encoder by taking the new encoder position and subtracts it from the old encoder postion and stores a sum of the difference. This also checks if the change in postion will overflow in either direction and accounts for that. '''
            encOld = self.encNew
            self.encNew = self.enctimer.counter()
            delta = self.encNew-encOld
            if delta>32768:
                delta = delta-65536
            elif delta<-32768:
                delta = delta+65536
            self.encTotal += delta
            return self.encTotal
                ## Current encoder postion is the sum of all the deltas.
        elif self.direction == 'counterclockwise':
            '''Calculates the current position of the encoder by taking the new encoder position and subtracts it from the old encoder postion and stores a sum of the difference. This also checks if the change in postion will overflow in either direction and accounts for that. '''
            encOld = self.encNew
            self.encNew = -self.enctimer.counter()
            delta = self.encNew-encOld
            if delta>32768:
                delta = delta-65536
            elif delta<-32768:
                delta = delta+65536
            self.encTotal += delta
            return self.encTotal
                ## Current encoder postion is the sum of all the deltas.
    def Velocity(self):
        if self.direction == 'clockwise':
            encOld = self.encNew
            self.encNew = self.enctimer.counter()
            delta = self.encNew-encOld
            if delta>32768:
                delta = delta-65536
            elif delta<-32768:
                delta = delta+65536
            return delta
        elif self.direction == 'counterclockwise':
            encOld = self.encNew
            self.encNew = -self.enctimer.counter()
            delta = self.encNew-encOld
            if delta>32768:
                delta = delta-65536
            elif delta<-32768:
                delta = delta+65536
            return delta
    def zero(self):
        '''Resets the encoder position to zero.'''
        self.encTotal = 0