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
    bus.write_byte_data(i2c_address, m2_dir, 1)
    bus.write_byte_data(i2c_address, m2_pwm, 80)
    bus.write_byte_data(i2c_address, m1_dir, 1)
    bus.write_byte_data(i2c_address, m1_pwm, 80)

    bus.write_byte_data(i2c_address, servo1, 0)
    bus.write_byte_data(i2c_address, servo2, 0)
    time.sleep(1)

    bus.write_byte_data(i2c_address, m2_pwm, 0)
    bus.write_byte_data(i2c_address, m1_pwm, 0)
    time.sleep(0.5)

    bus.write_byte_data(i2c_address, m2_dir, 0)
    bus.write_byte_data(i2c_address, m2_pwm, 80)
    bus.write_byte_data(i2c_address, m1_dir, 0)
    bus.write_byte_data(i2c_address, m1_pwm, 80)
    time.sleep(1)

    bus.write_byte_data(i2c_address, m2_pwm, 0)
    bus.write_byte_data(i2c_address, m1_pwm, 0)
    time.sleep(0.5)
 
    bus.write_byte_data(i2c_address, green_led, 1)
    bus.write_byte_data(i2c_address, m2_dir, 0)
    bus.write_byte_data(i2c_address, m2_pwm, 80)
    bus.write_byte_data(i2c_address, m1_dir, 1)
    bus.write_byte_data(i2c_address, m1_pwm, 80)
 
    time.sleep(1)
    bus.write_byte_data(i2c_address, green_led, 0)

    data = 50
    servo = 0
    while data < 150:
        bus.write_byte_data(i2c_address, m1_pwm, data)
        bus.write_byte_data(i2c_address, m2_pwm, data)
        bus.write_byte_data(i2c_address, servo1, servo)
        bus.write_byte_data(i2c_address, servo2, servo)
 
        servo += 1
        if servo == 14:
            servo = 0

        print("---")
    
        for i in range(10):
            val = bus.read_byte_data(i2c_address, i)
            print(f"{i} = {val}")
        
        time.sleep(0.2)
        data += 10
    
    bus.write_byte_data(i2c_address, m2_pwm, 0)
    bus.write_byte_data(i2c_address, m1_pwm, 0)
    time.sleep(3)

