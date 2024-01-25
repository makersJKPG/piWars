#!/usr/bin/env python3

import time
import smbus

i2c_address = 0x20
m1_dir = 0
m1_pwm = 1
m1_cnt_hi = 2
m1_cnt_lo = 3
m2_dir = 4
m2_pwm = 5
m2_cnt_hi = 6
m2_cnt_lo = 7
servo1 = 8
servo2 = 9
green_led = 10
blue_led = 11
m1_mode = 12
m2_mode = 13

bus = smbus.SMBus(1)

time.sleep(1)

data = 0
bus.write_byte_data(i2c_address, m1_dir, 0)
bus.write_byte_data(i2c_address, m2_dir, 0)
bus.write_byte_data(i2c_address, m1_pwm, data)
bus.write_byte_data(i2c_address, m2_pwm, data)
time.sleep(1)

# switch to pid speed mode
bus.write_byte_data(i2c_address, m1_mode, 0)
bus.write_byte_data(i2c_address, m2_mode, 0)

while True:
    data += 10
    if data > 250:
        data = 50
    bus.write_byte_data(i2c_address, m1_dir, 0)
    bus.write_byte_data(i2c_address, m2_dir, 0)
    bus.write_byte_data(i2c_address, m1_pwm, data)
    bus.write_byte_data(i2c_address, m2_pwm, data)
    print(f"pwm = {data}")
    hi = bus.read_byte_data(i2c_address, m1_cnt_hi)
    lo = bus.read_byte_data(i2c_address, m1_cnt_lo)
    val = hi*256 + lo
    print(f"m1_cnt = {val}")
    hi = bus.read_byte_data(i2c_address, m2_cnt_hi)
    lo = bus.read_byte_data(i2c_address, m2_cnt_lo)
    val = hi*256 + lo
    print(f"m2_cnt = {val}")
    time.sleep(1)
 
