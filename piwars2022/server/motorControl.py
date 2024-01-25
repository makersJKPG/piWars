
import smbus
import time

i2c_addr = 0x20
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

class motorControl:

    def __init__(self, disable_motors=False):

        self.speeds = [
            [0.0, 0.0],
            [0.2, 0.2],
            [0.3, 0.3],
            [0.4, 0.4],
            [0.5, 0.5],
            [0.6, 0.6],
        ]

        self.bus = smbus.SMBus(1)
        self.disable_motors = disable_motors

    def forward(self, speed, bearing = 0):
        A = self.speeds[speed][0]
        B = self.speeds[speed][1]
        self.throttle(A, B)

    def throttle(self, A, B, delay=None):

        if A > 1.0:
            A = 1.0
        if A < -1.0:
            A = -1.0
        if B > 1.0:
            B = 1.0
        if B < -1.0:
            B = -1.0

        gear_forward = 1
        gear_reverse = 0
        gear_neutral = 2
        gear_brake   = 3

        dir_a = gear_forward
        dir_b = gear_reverse    # b wheel direction reversed
        if A < 0:
            dir_a = gear_reverse
            A = -A
        if B < 0:
            dir_b = gear_forward
            B = -B
        if A == 0:
            dir_a = gear_brake
        if B == 0: 
            dir_b = gear_brake

        pwm_a = int(200 * A)
        pwm_b = int(180 * B)
        if not self.disable_motors:
            self.bus.write_byte_data(i2c_addr, m1_pwm, pwm_a)
            self.bus.write_byte_data(i2c_addr, m2_pwm, pwm_b)
            self.bus.write_byte_data(i2c_addr, m1_dir, dir_a)
            self.bus.write_byte_data(i2c_addr, m2_dir, dir_b)

            if delay is not None:
                time.sleep(delay)
                self.bus.write_byte_data(i2c_addr, m1_pwm, 0)
                self.bus.write_byte_data(i2c_addr, m2_pwm, 0)

    def release(self):
        self.bus.write_byte_data(i2c_addr, m1_pwm, 0)
        self.bus.write_byte_data(i2c_addr, m2_pwm, 0)
        self.release_servo() 

    def set_servo(self, id, value):
        print(f"### set_servo {id} {value}")
        
        if int(id) == 0:
            self.bus.write_byte_data(i2c_addr, servo1, int(value))
        else:
            self.bus.write_byte_data(i2c_addr, servo2, int(value))

    def set_servos(self, value1, value2):
        print(f"### set_servos {value1} {value2}")
        
        self.bus.write_byte_data(i2c_addr, servo1, int(value1))
        self.bus.write_byte_data(i2c_addr, servo2, int(value2))
        
    def open_gripper(self):
#        self.set_servo(200)
        self.set_servo(170)

    def close_gripper(self):
        self.set_servo(120)

    def release_servo(self):
        #self.kit._pca.channels[6].duty_cycle = 0
        None
       
    
