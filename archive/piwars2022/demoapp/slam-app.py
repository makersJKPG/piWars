#!/usr/bin/env python3

import cv2
import numpy as np
import time
import math

from camControl import *
from slamProcessor import *

proc = slamProcessor({
    "roixmin": 0,
    "roixmax": 100,
    "roiymin": 0,
    "roiymax": 100,
    "scale_percent": 100,
    "dot_max_area": 200
})
cam = camControl(vflip=True, showSource=False)
cam.registerCallback(proc.process)

while True:

    time.sleep(3)
    #print("running")
