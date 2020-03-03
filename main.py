'''
main
'''

'''
Keys for getting TOF to operate
1. driver program utilized the machine library
2. I2C initialization is based on which bus is used, pins are automatically initialized
'''
from VL53L0X import VL53L0X
from machine import I2C
i2c = I2C(1,freq = 200000)
TOF1 = VL53L0X(i2c)
TOF1.start()
TOF1.read()
print('done TOF')

'''
MPU 
'''
