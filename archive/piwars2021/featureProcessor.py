import numpy as np
import cv2

class featureProcessor:

    def  __init__(self):

        self.green = None
        self.red = None
        self.blue = None

        # Initiate FAST object with default values
        self.fast = cv2.FastFeatureDetector_create()
        print("Init featureProcessor")

    def process(self, img):

        height, width, _ = img.shape

        img = img[height//2:height,:]
        height, width, _ = img.shape

        self.frameHeight = height
        self.frameWidth = width

        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        ret, mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

        # find and draw the keypoints
        kp = self.fast.detect(mask, None)
        img2 = img.copy()
        img2 = cv2.drawKeypoints(img, kp, img2, color = (255,0,0))

        # Print all default params
        print("Threshold: {}".format(self.fast.getThreshold()))
        print("nonmaxSuppression: {}".format(self.fast.getNonmaxSuppression()))
        print("neighborhood: {}".format(self.fast.getType()))
        print("Total Keypoints with nonmaxSuppression: {}".format(len(kp)))

        # Disable nonmaxSuppression
        #fast.setBool('nonmaxSuppression',0)
        #kp = fast.detect(img,None)
        #print "Total Keypoints without nonmaxSuppression: ", len(kp)
        #img3 = cv2.drawKeypoints(img, kp, color=(255,0,0))

        cv2.imshow("img2", img2)
        cv2.imshow("mask", mask)
        key = cv2.waitKey(1)
        if key == 'q':
            print("Q")
            return True
        else:
            return False
