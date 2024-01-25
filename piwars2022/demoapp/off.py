#!/usr/bin/env python3

from my_motorkit import MotorKit

from time import sleep

kit = MotorKit(pwm_frequency=50)


def throttle(A, B, delay=None):
    #print("speed={},{}".format(A, B))
    kit.motor1.throttle = A
    kit.motor2.throttle = B
    if delay is not None:
        sleep(delay)

def release():
    kit.motor1.throttle = None
    kit.motor2.throttle = None

release()

