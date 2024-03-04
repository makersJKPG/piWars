import cv2
import numpy as np

from enum import Enum

class cubeType(Enum):
    RED = 1
    GREEN = 2
    BLUE = 3

#green_hue = 41     # in sunlight
green_hue = 65    # in lab
#blue_hue = 110
blue_hue = 16
#red_hue = 173
red_hue = 122

yellow_hue = 99
yellow_sat = 228
yellow_lum = 196

#cube_detect_threshold = 200
cube_detect_threshold = 100

class cubeUtil:

    @staticmethod
    def getCubePos(img_hsv, color, roi=None):
        if color == "green":
            return cubeUtil.cubePos(img_hsv, green_hue, "green", roi=roi)
        elif color == "blue":
            return cubeUtil.cubePos(img_hsv, blue_hue, "blue", roi=roi)
        elif color == "red":
            return cubeUtil.cubePos(img_hsv, red_hue, "red", roi=roi)
        elif color == "yellow":
            return cubeUtil.cubePos(img_hsv, yellow_hue, "yellow", roi=roi)
        else:
            print("Invalid color {}".format(color))
            return None

    @staticmethod
    def getRedPos(img_hsv, roi=None):
        return cubeUtil.cubePos(img_hsv, red_hue, "red", roi=roi)

    @staticmethod
    def getGreenPos(img_hsv, roi=None):
        return cubeUtil.cubePos(img_hsv, green_hue, "green", roi=roi)

    @staticmethod
    def getBluePos(img_hsv, roi=None):
        return cubeUtil.cubePos(img_hsv, blue_hue, "blue", roi=roi)

    @staticmethod
    def cubePos(img_hsv, hue, name, roi=None):

        height, width, _ = img_hsv.shape

        if roi is not None:
            # mask out vertically from roi % to bottom of image...
            img_hsv[int(height*roi/100.0):height-1,0:width-1] = (0, 0, 0)

        mask = cubeUtil.mask(img_hsv, hue)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        #mask = cv2.dilate(mask, kernel, iterations=1)

        cv2.imshow("mask", mask)
        pos = cubeUtil.boxPos(mask) if cv2.countNonZero(mask) > cube_detect_threshold else None
        return pos

    @staticmethod
    def mask(img, hue):
        lo = (hue - 13, 150, 35)
        hi = (hue + 13, 255, 255)
        mask = cv2.inRange(img, lo, hi)
        return mask

    @staticmethod
    def boxPos(mask):
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
