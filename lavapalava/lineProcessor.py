import cv2
import piwars2024
from pid import *
import numpy as np
import time

class lineProcessor:

    def  __init__(self, params=None):

        size = 300, 300, 3
        m = np.zeros(size, dtype=np.uint8) 
        cv2.imshow("image", m)
        size = 300, 300
        m = np.zeros(size, dtype=np.uint8)
        cv2.imshow("mask", m)

        cv2.moveWindow("image", 0, 0)
        cv2.moveWindow("mask", 0, 280)

        limMin = -1.0
        limMax = 1.0
        limMinInt = -1.0
        limMaxInt = 1.0
        T = 0.02
        tau = 1.0

        self.pid = pid(0.01, 0.0, 0.00, T, tau, limMin, limMax, limMinInt, limMaxInt)

        if params is not None:
            self.params = params
        else:
            self.params = {
                "disable_motors": False,
                "roixmin": 0,
                "roixmax": 100,
                "roiymin": 0,
                "roiymax": 100,
                "scale_percent": 100,
                "line_detect_threshold": 100
            }

        piwars2024.set_mode(1)

        self.start_time = time.time()

    def get_roi(self, img):

        imgh, imgw = img.shape

        # crop image...
        roixmin = int(imgw * self.params["roixmin"] / 100.0)
        roixmax = int(imgw * self.params["roixmax"] / 100.0)
        roiymin = int(imgh * self.params["roiymin"] / 100.0)
        roiymax = int(imgh * self.params["roiymax"] / 100.0)
        img = img[roiymin:roiymax,roixmin:roixmax]
        
        # scale image...
        width = int(img.shape[1] * self.params["scale_percent"] / 100.0)
        height = int(img.shape[0] * self.params["scale_percent"] / 100.0)
        resized = cv2.resize(img, (width, height))
        
        return resized

    def process(self, img):

        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        img = self.get_roi(img)

        height, width = img.shape

        self.frameHeight = height
        self.frameWidth = width

        ret, mask = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY_INV)
        self.line = self.boxPos(mask) if cv2.countNonZero(mask) > self.params["line_detect_threshold"] else None

        if self.line is not None:
            cv2.circle(img, self.line, 5, 127, 1)

        cv2.imshow("mask", mask)
        cv2.imshow("image", img)

        measurement = self.get_measurement(mask)
        setpoint = 0

        if measurement is not None:
            output = self.pid.update(setpoint, measurement)
            adj_r = -output if output < 0 else 0
            adj_l = output if output > 0 else 0
            print("measurement={}, output={}".format(measurement, output))
            ## self.motor.throttle(1.0 - adj_l, 1.0 - adj_r)
        else:
            pass
            ## self.motor.throttle(0, 0)

        key = cv2.waitKey(1)
        if key == 'q':
            print("Q")
            return True
        else:
            return False

    def boxPos(self, mask):
        # calculate moments of binary image
        M = cv2.moments(mask)
        # calculate x,y coordinate of center
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX = 0
            cY = 0
        return cX, cY

    def get_measurement(self, img):

        if self.line is not None:
            return self.line[0] - self.frameWidth/2
        else:
            return None
