import cv2
import numpy as np
from picamera2 import Picamera2

from lineProcessor import *

proc = lineProcessor({
    "disable_motors": False,
    "roixmin": 0,
    "roixmax": 100,
    "roiymin": 70,
    "roiymax": 100,
    "scale_percent": 100,
    "line_detect_threshold": 1000
})

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

while True:
    im = picam2.capture_array()
    #hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    im = cv2.flip(im, -1)
    cv2.imshow('image', im)

    proc.process3(im)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()

