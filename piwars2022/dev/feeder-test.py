#!/usr/bin/env python3

import time
import smbus

i2c_address = 0x20
m1_dir = 0
m1_pwm = 1
m1_cnthi = 2
m1_cntlo = 3
m2_dir = 4
m2_pwm = 5
m2_cnthi = 6
m2_cntlo = 7
servo1 = 8
servo2 = 9
green_led = 10
blue_led = 11

bus = smbus.SMBus(1)

time.sleep(1)

while True:

    bus.write_byte_data(i2c_address, servo1, 12)
    bus.write_byte_data(i2c_address, servo2, 0)
    time.sleep(1)
    bus.write_byte_data(i2c_address, servo1, 8)
    bus.write_byte_data(i2c_address, servo2, 2)
    time.sleep(1)
    bus.write_byte_data(i2c_address, servo1, 6)
    bus.write_byte_data(i2c_address, servo2, 4)
    time.sleep(1)
    bus.write_byte_data(i2c_address, servo1, 4)
    bus.write_byte_data(i2c_address, servo2, 6)
    time.sleep(1)

