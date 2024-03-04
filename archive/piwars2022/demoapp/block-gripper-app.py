#!/usr/bin/env python3

import cv2
import numpy as np
import time
import math

from camControl import *
from cubeProcessor import *

proc = cubeProcessor()
cam = camControl(showSource=True)
cam.registerCallback(proc.process)

while True:

    time.sleep(3)
    print("running")

    start = 0x0900
    delta = 0x0016

    angle = 120
    kit._pca.channels[14].duty_cycle = delta * angle + start
    sleep(1)

    angle = 200
    kit._pca.channels[14].duty_cycle = delta * angle + start
    sleep(1)

