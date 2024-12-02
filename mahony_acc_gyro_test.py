import utime
from machine import I2C
import lsm9ds1
import mahony_acc_gyro

## create the ahrs_filter
ahrs_filter = mahony_acc_gyro.Mahony(50, 5, 100)

bus = I2C(1)
sensor = lsm9ds1.LSM9DS1(bus)

# used to count how often we are feeding the ahrs_filter
count = 0

lastPrint = utime.ticks_ms()
timestamp = utime.ticks_us()

while True:
    
    # 10000us = 10ms = 100Hz
    if (utime.ticks_us() - timestamp) > 10000:

        gx, gy, gz = sensor.read_gyro()
        ax, ay, az = sensor.read_accel()
        ahrs_filter.update_IMU(gx, gy, -gz, ax, ay, az)

        count += 1
        timestamp = utime.ticks_us()

    # every 100ms print the ahrs_filter values
    if (utime.ticks_ms()) > lastPrint + 100:
        # radians multiply by 57.20578 to degrees
        print(
            "Orientation: ",
            ahrs_filter.pitch * 57.29578, ", ",
            ahrs_filter.roll * 57.29578)

        #print("Count: ", count)  
        count = 0     # reset count
        lastPrint = utime.ticks_ms()
     
