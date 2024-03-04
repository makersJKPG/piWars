import cv2
import numpy as np
import time

from motorControl import *

from camControl import *
from cubeProcessor import *
#proc = cubeProcessor()
#cam = camControl()
#cam.registerCallback(proc.process)
#time.sleep(3)

motor = motorControl()

#while cam.isAlive():
while True:
    
    #motor.throttle(0.35, 0.385)
    motor.forward(1)
    time.sleep(3)
    break


motor.throttle(None, None)
