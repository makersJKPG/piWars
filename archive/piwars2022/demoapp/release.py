#!/usr/bin/env python3

from adafruit_motorkit import MotorKit

from time import sleep

kit = MotorKit(pwm_frequency=50)


def throttle(A, B, delay=None):
    print("speed={},{}".format(A, B))
    kit.motor1.throttle = -A
    kit.motor2.throttle = -B
    if delay is not None:
        sleep(delay)

def release():
    kit.motor1.throttle = None
    kit.motor2.throttle = None

#throttle(0.2, 0.2)

start = 0x0900
delta = 0x0016

kit._pca.channels[14].duty_cycle = 0
#sleep(1)

#angle = 200
#kit._pca.channels[14].duty_cycle = delta * angle + start
#sleep(1)





