import io
import time
import threading
import picamera

import numpy as np
import cv2

from cubeProcessor import *

# Create a pool of image processors
done = False
lock = threading.Lock()
pool = []

#width = 1280
#height = 720

#width = 1648
#height = 928

#width = 1296
#height = 736

width = 640
height = 480

class ImageProcessor(threading.Thread):
    def __init__(self):
        super(ImageProcessor, self).__init__()
        self.image = np.empty((height, width, 3), dtype=np.uint8)
        self.event = threading.Event()
        self.terminated = False
        self.start()

        self.processor = cubeProcessor()

    def run(self):
        # This method runs in a separate thread
        global done
        while not self.terminated:
            if self.event.wait(1):
                try:
                    result, mask = self.processor.process(self.image)
                    
                    #cv2.imshow("frame", self.image)
                    cv2.imshow("result", result)
                    cv2.imshow("mask", mask)

                    key = cv2.waitKey(1)
                    if key == 'q':
                        done = True
                finally:
                    # Reset the event
                    self.event.clear()
                    # Return ourselves to the pool
                    with lock:
                        pool.append(self)

starving = False

def streams():
    while not done:
        with lock:
            if pool:
                processor = pool.pop()
            else:
                processor = None
        if processor:
            starving = False
            yield processor.image
            processor.event.set()
        else:
            if not starving:
                print("waiting for image processor")
                starving = True
            time.sleep(0.01)

with picamera.PiCamera() as camera:
    pool = [ImageProcessor() for i in range (4)]
    print("pool={}".format(len(pool)))
    camera.resolution = (width, height)
    # Set the framerate appropriately; too fast and the image processors
    # will stall the image pipeline and crash the script
    camera.framerate = 10
    camera.raw_format = 'rgb'
    camera.resolution = (width, height)
    camera.vflip = True
    #camera.start_preview()
    #time.sleep(2)
    camera.capture_sequence(streams(), format="raw", use_video_port=True)

# Shut down the processors in an orderly fashion
while pool:
    with lock:
        processor = pool.pop()
    processor.terminated = True
    processor.join()
