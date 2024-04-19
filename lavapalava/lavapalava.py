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
   
    imgh, imgw, _ = img.shape
    
    # crop image...
    roixmin = int(imgw * params["roixmin"])
    roixmax = int(imgw * params["roixmax"])
    roiymin = int(imgh * params["roiymin"])
    roiymax = int(imgh * params["roiymax"])
    img = img[roiymin:roiymax,roixmin:roixmax]
    
    if params["scale_percent"]:
        # scale image...
        width = int(img.shape[1] * params["scale_percent"] / 100.0)
        height = int(img.shape[0] * params["scale_percent"] / 100.0)
        img = cv2.resize(img, (width, height))

    #print("{}x{}".format(width, height))
    return img

def add_blinkers(img, shade):

    height, width = img.shape

    pt1 = (int(width*0.66), 0)
    pt2 = (width-1, 0)
    pt3 = (width-1, int(height*0.5))
    triangle_cnt = np.array( [pt1, pt2, pt3] )
    cv2.drawContours(img, [triangle_cnt], 0, (shade, shade, shade), -1)

    pt1 = (int(width*0.33), 0)
    pt2 = (0, 0)
    pt3 = (0, int(height*0.5))
    triangle_cnt = np.array( [pt1, pt2, pt3] )
    cv2.drawContours(img, [triangle_cnt], 0, (shade, shade, shade), -1)

    cv2.rectangle(img, 
                  (int(width/2-width*0.3), int(height*0.6)), 
                  (int(width/2+width*0.3), int(height-1)), 
                  (shade, shade, shade), -1)

    return img

def find_colour(img, hue):

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    red_only = cv2.bitwise_and(img, img, mask=mask)

    return red_only

offset = 0.0

def image_process(img):
    global speed, turning, offset

    height, width = img.shape

    # dark line
    #ret, mask = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY_INV)
    
    # bright line
    ret, mask = cv2.threshold(img, 180, 255, cv2.THRESH_BINARY)
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

#        output = pid.update(setpoint, measurement)
#        offset += output
#        if offset > 25:
#            offset = 25.0
#        if offset < -25:
#            offset = -25.0
#        lval = int(round(speed + offset))
#        rval = int(round(speed - offset))

        print("setpoint={}, measurement={}, error={}, output={}, lval={}, rval={}".format(setpoint, measurement, pid.prevError, output, lval, rval)) 

        piwars2024.set_speed(lval, rval)
    else:
        piwars2024.set_speed(0, 0)
        #pass

    return mask

def wait_for_start_button():
    count = 0
    while True:
        time.sleep(0.1)
        print("Waiting {}".format(count))
        count = count + 1
        while jr.events_available() != 0:
            event = jr.get_event()
            if event["value"] == -32767 and event["action"] == 2 and event["button"] == 7:
                print("pressed up")
                return
            else:
                print("nothing to do {}, {}, {}, {}".format(event["milli"], event["value"], event["action"], event["button"]))

params = {
    "disable_motors": False,
    "roixmin": 0.0,
    "roixmax": 1.0,
    "roiymin": 0.25,
    "roiymax": 1.0,
    "scale_percent": 100,
    "line_detect_threshold": 1000
}

limMin = -60.0
limMax = 60.0
limMinInt = -1.0
limMaxInt = 1.0
T = 0.02
tau = 0.0
pid = piwars2024.pid(0.2, 0.02, 0.00, T, tau, limMin, limMax, limMinInt, limMaxInt)

jr = piwars2024.JoyReader()

picam2 = Picamera2()
#picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (1920, 1080)}))
picam2.configure(picam2.create_preview_configuration(
    main = {
        "format": 'RGB888', 
        #"size": picam2.sensor_resolution
        "size": (1920, 1080)
    },
    transform = libcamera.Transform(hflip=1, vflip=1)    
))
picam2.start()

print("Started camera")

size = 300, 300, 3
m = np.zeros(size, dtype=np.uint8) 
cv2.imshow("image", m)
size = 300, 300
m = np.zeros(size, dtype=np.uint8)
cv2.imshow("mask", m)

piwars2024.set_mode(1)
time.sleep(4)
# turn off auto exposure after setting has stabilized
picam2.set_controls({'AeEnable': False})

wait_for_start_button()

speed = -35
turning = 0

piwars2024.accelerate(0, speed, steps=4, intervaltime=0.1)
running = True

while running:
    img = picam2.capture_array()
    img = get_roi(img)

    #colmask = find_colour(img, 55)
    grey = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    grey = add_blinkers(grey, 0)

    #print(colmask.shape)
    mask = image_process(grey)
    #cv2.imshow("colmask", colmask)
    cv2.imshow("mask", mask)
    cv2.imshow('image', grey)

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

