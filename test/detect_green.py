import cv2
import numpy as np
from picamera2 import Picamera2

class detect_green:
    def __init__(self):
        self.picam2 = Picamera2()
        self.picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
        self.picam2.start()
        self.im=[]
        self.green_only=[]

    def run(self):
        while True:
            im = self.picam2.capture_array()

            hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

            lower_green = np.array([35, 50, 50])
            upper_green = np.array([85, 255, 255])

            mask = cv2.inRange(hsv, lower_green, upper_green)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            self.green_only = cv2.bitwise_and(im, im, mask=mask)

            cv2.imshow('Camera', green_only)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
        picam2.stop()

