#from adafruit_motorkit import MotorKit
from my_motorkit import MotorKit
import time

class motorControl:

    def __init__(self, disable_motors=False):
        self.kit = MotorKit(pwm_frequency=50, address=0x60)

        self.speeds = [
            [0.0, 0.0],
            [0.2, 0.2],
            [0.3, 0.3],
            [0.4, 0.4],
            [0.5, 0.5],
            [0.6, 0.6],
        ]

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

        if not self.disable_motors:
            self.kit.motor1.throttle = A
            self.kit.motor2.throttle = B

            if delay is not None:
                time.sleep(delay)
                self.kit.motor1.throttle = 0
                self.kit.motor2.throttle = 0

    def release(self):
        self.kit.motor1.throttle = None
        self.kit.motor2.throttle = None
        self.release_servo() 

    def set_servo(self, value):
        start = 0x0900
        delta = 0x0016

        #pca_channel = 14 # channel used on motor hat
        pca_channel = 6 # channel used on piwars board

        # range value 120 to 200
        self.kit._pca.channels[pca_channel].duty_cycle = delta * value + start

    def open_gripper(self):
#        self.set_servo(200)
        self.set_servo(170)

    def close_gripper(self):
        self.set_servo(90)

    def release_servo(self):
        self.kit._pca.channels[6].duty_cycle = 0

       
    
