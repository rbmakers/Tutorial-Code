import time
from machine import Pin, I2C
from lsm9ds1 import LSM9DS1

bus = I2C(1)
lsm = LSM9DS1(bus)
while (True):

    #print("姿態感測器讀值 : ")
    #print("ACC : ", lsm.read_accel())
    #print("GYRO : ", lsm.read_gyro())
    #print("Magnet : ", lsm.read_magnet())
    print(lsm.read_accel())
    print(lsm.read_gyro())
    print(lsm.read_magnet())
    #print("         ")
    
    time.sleep_ms(1000)