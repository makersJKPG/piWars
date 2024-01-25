from adafruit_motorkit import MotorKit

from time import sleep

kit = MotorKit()


def throttle(A, B, delay=None):
    #print("speed={},{}".format(A, B))
    kit.motor1.throttle = A
    kit.motor2.throttle = B
    if delay is not None:
        sleep(delay)

def release():
    kit.motor1.throttle = None
    kit.motor2.throttle = None


#for i in range(0, 110, 10):
#    ss(i/100, i/100)
#    time.sleep(0.05)

#throttle(0.5, 0.5, 0.1)
throttle(1.0, 1.0, 1)
throttle(0.25, 0.25, 3)
throttle(-0.5, -0.5, 3)
#hrottle(0.5, 0.5, 0.1)

#for i in range(100, 0, -10):
#    ss(i/100, i/100)
#    time.sleep(0.05)

release()

