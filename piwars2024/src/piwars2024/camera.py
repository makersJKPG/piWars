import time
import spidev
import struct
import cv2 as cv
import numpy as np
from picamera2 import Picamera2
from threading import Thread as t
from time import sleep

class camera:
    def __init__(self,w=640,h=480):
        self.cam=Picamera2()
        self.frame=[]
        self.camThread=t(target=self.runCam,args=(),daemon=True)
        self.cam.configure(self.cam.create_preview_configuration(main={"format": 'XRGB8888', "size": (w, h)}))
        self.cam.start()
        self.camThread.start()
        self.size=(w,h)
    def runCam(self):
        while True:
            self.frame=self.cam.capture_array()
    def getFrame(self):
        return self.frame
    def destroy(self):
        self.camThread=0
        self.cam.stop()

c=camera()
sleep(1)
while True:
    frame=c.getFrame()
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    lower_green = np.array([35, 50, 50])
    upper_green = np.array([85, 255, 255])

    mask = cv.inRange(hsv, lower_green, upper_green)

    

    kernel = np.ones((5, 5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    green_only = cv.bitwise_and(frame, frame, mask=mask)

    cv.imshow('Camera', green_only)
    


    if cv.waitKey(1) & 0xFF == ord('q'):
        break

c.destroy()

