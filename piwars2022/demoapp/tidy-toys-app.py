#!/usr/bin/env python3

import cv2
import numpy as np
import time
import math

from camControl import *
from cubeProcessor import *

# tidy up the toys...
proc = cubeProcessor()
cam = camControl()
cam.registerCallback(proc.process)

while proc.shutdown == False:

    time.sleep(0.25)


cam.stop()
