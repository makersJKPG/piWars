import cv2
import numpy as np
import time
import math

from motorControl import *

from camControl import *
from cubeProcessor import *

proc = cubeProcessor()

cam = camControl()
cam.registerCallback(proc.process)

motor = motorControl()

time.sleep(3)

searching = True

#while cam.isAlive():
while True:
    
    #if proc.green is not None:
    #    print("green {}".format(proc.green))
    #if proc.blue is not None:
    #    print("blue {}".format(proc.blue))
    #if proc.red is not None:
    #    print("red {}".format(proc.red))

    #print("colorpicker: {}".format(proc.colorpicker))

    red = proc.red
    if red is None:
        print("Lost")
        searching = True
        motor.throttle(0.3, -0.3)
    else:
        redx = red[0]
        redy = red[1]

        if searching:
            print("Found")
            searching = False
#            motor.throttle(-0.5, 0.5)
#            time.sleep(0.1)
            motor.throttle(0, 0)
            time.sleep(0.5)
        
        print("x={}, y={}".format(redx, redy))

        if redy < 120:
            xpos = redx - proc.frameWidth//2
            if xpos > 40:
                motor.throttle(0.3, -0.3)
                time.sleep(0.05)
                motor.throttle(0, 0)
                time.sleep(0.2)
            elif xpos < -40:
                motor.throttle(-0.3, 0.3)
                time.sleep(0.05)
                motor.throttle(0, 0)
                time.sleep(0.2)
            else:
                print("Tracking")
                motor.throttle(0.35, 0.385)
        else:
            print("Stop")
            motor.throttle(0, 0)

