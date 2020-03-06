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
import utime

i2c = I2C(1,freq = 200000) #Uses bus 3 because of the pins it is connected to
i2c2 = I2C(3,freq = 200000)
TOF1 = VL53L0X(i2c)
TOF2 = VL53L0X(i2c2)
#TOF1.start()
#TOF1.read()
print('done TOF')


'''
MPU 
'''
from imu import MPU6050
#i2c = I2C(1,freq = 200000)
imu = MPU6050(i2c)
#print(imu.accel.xyz)

while True:
    print(imu.accel.xyz)
    TOF1.read()
    TOF2.read()
    #print(imu.gyro.xyz)

    #print(imu.accel.z)
    utime.sleep(1)
