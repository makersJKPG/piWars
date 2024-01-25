import cv2
import numpy as np
import time
import math

from motorControl import *

from camControl import *
from featureProcessor import *

proc = featureProcessor()

cam = camControl()
cam.registerCallback(proc.process)

motor = motorControl()

time.sleep(3)

while True:
    
    time.sleep(3)
    print("processing")

