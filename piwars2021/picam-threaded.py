import io
import time
import threading
import picamera

import numpy as np
import cv2

# Create a pool of image processors
done = False
lock = threading.Lock()
pool = []

class ImageProcessor(threading.Thread):
    def __init__(self):
        super(ImageProcessor, self).__init__()
        self.stream = io.BytesIO()
        self.event = threading.Event()
        self.terminated = False
        self.start()

    def run(self):
        # This method runs in a separate thread
        global done
        while not self.terminated:
            if self.event.wait(1):
                try:
                    self.stream.seek(0)
                    w = 640
                    h = 480
                    frame = cv2.imdecode(np.fromstring(self.stream.getvalue(), dtype=np.uint8), 1)
                    cv2.imshow("frame", frame)
                    key = cv2.waitKey(1)
                    #if key == 'q':
                    #    done = True
                    # Read the image and do some processing on it
                    #Image.open(self.stream)
                    #...
                    #...
                    # Set done to True if you want the script to terminate
                    # at some point
                    #done=True
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
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
            yield processor.stream
            processor.event.set()
        else:
            if not starving:
                print("waiting for image processor")
                starving = True
            time.sleep(0.01)

with picamera.PiCamera() as camera:
    pool = [ImageProcessor() for i in range (4)]
    print("pool={}".format(len(pool)))
    camera.resolution = (640, 480)
    # Set the framerate appropriately; too fast and the image processors
    # will stall the image pipeline and crash the script
    camera.framerate = 10
    #camera.start_preview()
    #time.sleep(2)
    camera.capture_sequence(streams(), format="rgb", use_video_port=True)

# Shut down the processors in an orderly fashion
while pool:
    with lock:
        processor = pool.pop()
    processor.terminated = True
    processor.join()
