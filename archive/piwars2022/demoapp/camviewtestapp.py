#!/usr/bin/env python3

import cv2
import numpy as np
import time
import math

from camControl import *
from lineProcessor import *

# basic line follower...
#proc = lineProcessor()
#cam = camControl()
#cam.registerCallback(proc.process)

# up the garden path...
proc = lineProcessor({
    "disable_motors": False,
    "roixmin": 0,
    "roixmax": 100,
    "roiymin": 70,
    "roiymax": 100,
    "scale_percent": 100,
    "line_detect_threshold": 1000
})
cam = camControl(False, True)

while True:

    time.sleep(3)
    #print("running")
