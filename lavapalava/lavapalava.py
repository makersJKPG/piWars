import math
import time
import struct
import cv2
import numpy as np
from picamera2 import Picamera2
import libcamera
import piwars2024

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

def get_roi(img):

    imgh, imgw = img.shape
    
    # crop image...
    roixmin = int(imgw * params["roixmin"])
    roixmax = int(imgw * params["roixmax"])
    roiymin = int(imgh * params["roiymin"])
    roiymax = int(imgh * params["roiymax"])
    img = img[roiymin:roiymax,roixmin:roixmax]
    
    # scale image...
    width = int(img.shape[1] * params["scale_percent"] / 100.0)
    height = int(img.shape[0] * params["scale_percent"] / 100.0)
    resized = cv2.resize(img, (width, height))
    
    return resized

def image_process(img):
    global speed, turning

    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    img = get_roi(img)

    height, width = img.shape

    #ret, mask = cv2.threshold(img, 110, 255, cv2.THRESH_BINARY_INV)
    ret, mask = cv2.threshold(img, 150, 255, cv2.THRESH_BINARY)
    pos = boxPos(mask) if cv2.countNonZero(mask) > params["line_detect_threshold"] else None

    if pos is not None:
        cv2.circle(img, pos, 5, 127, 1)

        x = pos[0]
        y = pos[1]
        measurement = x - width/2
        setpoint = 0

        # pid regulates a turning angle / direction which should nominally be 0 degrees
        output = pid.update(setpoint, measurement)

        angle = output*math.pi/2/100
        scale = math.cos(angle)
        if angle > 0:
            lval = int(round(speed * scale))
            rval = int(speed)
        else:
            lval = int(speed)
            rval = int(round(speed * scale))

        print("setpoint={}, measurement={}, error={}, output={}, angle={}, lval={}, rval={}".format(setpoint, measurement, pid.prevError, output, angle, lval, rval)) 

        piwars2024.set_speed(lval, rval)
    else:
        piwars2024.set_speed(0, 0)
        #pass

    cv2.imshow("mask", mask)
    cv2.imshow("image", img)

params = {
    "disable_motors": False,
    "roixmin": 0.0,
    "roixmax": 1.0,
    "roiymin": 0.7,
    "roiymax": 1.0,
    "scale_percent": 100,
    "line_detect_threshold": 1000
}

size = 300, 300, 3
m = np.zeros(size, dtype=np.uint8) 
cv2.imshow("image", m)
size = 300, 300
m = np.zeros(size, dtype=np.uint8)
cv2.imshow("mask", m)

cv2.moveWindow("image", 0, 0)
cv2.moveWindow("mask", 0, 280)

limMin = -45.0
limMax = 45.0
limMinInt = -1.0
limMaxInt = 1.0
T = 0.02
tau = 0.0

pid = piwars2024.pid(0.075, 0.00, 0.00, T, tau, limMin, limMax, limMinInt, limMaxInt)

jr = piwars2024.JoyReader()

picam2 = Picamera2()
#picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (1920, 1080)}))
picam2.configure(picam2.create_preview_configuration(
    main = {
        "format": 'XRGB8888', 
        "size": picam2.sensor_resolution
        #"size": (1920, 1080)
    },
    transform = libcamera.Transform(hflip=1, vflip=1)    
))
picam2.start()

print("Started camera")

piwars2024.set_mode(1)
time.sleep(2)

piwars2024.accelerate(0, -10, steps=3, intervaltime=0.1)
speed = -20
turning = 0

running = True

while running:
    im = picam2.capture_array()
    #hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    #im = cv2.flip(im, -1)
    cv2.imshow('image', im)

    #image_process(im)

    while jr.events_available() != 0:
        event = jr.get_event()
        if event["value"] == -32767 and event["action"] == 2 and event["button"] == 7:
            print("pressed up")
            running = False
#        else:
#            print("no to do {}, {}, {}, {}".format(event["milli"], event["value"], event["action"], event["button"]))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        running = False

piwars2024.set_speed(0, 0)
piwars2024.set_motor(0, 0, 0, 0)
time.sleep(0.5)
piwars2024.set_mode(2) # PWM mode

cv2.destroyAllWindows()
picam2.stop()

jr.stop()

exit(0)

