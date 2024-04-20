import math
import time
import struct
import cv2
import numpy as np
from picamera2 import Picamera2
import piwars2024
import imuread

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

    ret, mask = cv2.threshold(img, 110, 255, cv2.THRESH_BINARY_INV)
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
        piwars2024.set_speed(-10, 10)
        #pass

    cv2.imshow("mask", mask)
    cv2.imshow("image", img)

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

limMin = -30.0
limMax = 30.0
limMinInt = -1.0
limMaxInt = 1.0
T = 0.02
tau = 0.0
imu_pid = piwars2024.pid(0.10, 0.00, 0.00, T, tau, limMin, limMax, limMinInt, limMaxInt)

jr = piwars2024.JoyReader()

imu = imuread.ImuReader()

speed = -10
turning = 0

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (1920, 1080)}))
picam2.start()

print("Calibrating IMU...")
time.sleep(0.5)
piwars2024.set_motor(0, 0, 0, 0)
piwars2024.set_mode(2)
time.sleep(0.5)
piwars2024.calibrate_imu()
time.sleep(25)
print("DONE")

wait_for_start_button()

#piwars2024.imu_turn(180)
#for i in range(0, 25):
#    print("yaw={}".format(imu.yaw))
#    time.sleep(0.5)
#piwars2024.imu_turn(90)
#for i in range(0, 25):
#    print("yaw={}".format(imu.yaw))
#    time.sleep(0.5)
#piwars2024.imu_turn(0)
#for i in range(0, 25):
#    print("yaw={}".format(imu.yaw))
#    time.sleep(0.5)
#piwars2024.imu_turn(-90)
#for i in range(0, 25):
#    print("yaw={}".format(imu.yaw))
#    time.sleep(0.5)
#piwars2024.imu_turn(0)
#for i in range(0, 25):
#    print("yaw={}".format(imu.yaw))
#    time.sleep(0.5)

piwars2024.set_mode(1) # speed control mode
piwars2024.set_speed(0, 0)

lval = 0
rval = 0
piwars2024.set_speed(0, 0)

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

    return img
 
def turn(degrees):
    setpoint = imu.yaw + degrees
    speed = 0.0
    piwars2024.set_speed(0, 0)
    while abs(setpoint - imu.yaw) > 1.0:
        measurement = imu.yaw
        # pid regulates a turning angle / direction which should nominally be 0 degrees
        output = imu_pid.update(setpoint, measurement)
        speed = speed + output
        if speed > 255:
            speed = 255
        if speed < -255:
            speed = -255
        print("setpoint={}, measurement={}, error={}, output={}".format(setpoint, measurement, imu_pid.prevError, output))
        piwars2024.set_speed(int(round(output)), -int(round(output)))
        time.sleep(0.01)
    piwars2024.set_speed(0, 0)

def drive_distance(distance, direction):
    piwars2024.set_speed(0, 0)
    time.sleep(0.5)
    piwars2024.reset_encoder()
    m1 = 0
    m2 = 0
    piwars2024.set_speed(-30, -30)
    while abs(m1) < distance:
        m1, m2 = piwars2024.get_encoder()
        print("m1={}, m2={}, yaw={}".format(m1, m2, imu.yaw))
    piwars2024.set_speed(0, 0)

def drive_until_blocked():

    print("1")
    piwars2024.set_speed(-40, -40)
    piwars2024.set_mode(1)

    blocked = False
    while not blocked:
        img = picam2.capture_array()
        #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img = cv2.flip(img, -1)

        # make grayscale
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        # crop image to the area which shouldn't be blocked
        imgh, imgw = img.shape 
        roixmin = int(imgw * 0.25)
        roixmax = int(imgw * 0.75)
        roiymin = int(imgh * 0.3)
        roiymax = int(imgh * 0.5)
        img = img[roiymin:roiymax,roixmin:roixmax]
    
        img = add_blinkers(img, 0)

        # scale image...
        width = 100
        height = 100
        resized = cv2.resize(img, (width, height))

        cv2.imshow('image', img)
       
        # threshold area to mask
        ret, mask = cv2.threshold(img, 220, 255, cv2.THRESH_BINARY)
        cv2.imshow('mask', mask)

        # count pixels in mask
        # if pixels greater than a limit then stop running
        numpix = cv2.countNonZero(mask)
        print(numpix)
        if numpix > 5000:
            blocked = True
        
        cv2.waitKey(1)

    piwars2024.set_speed(0, 0)

# tactic:
# drive forward x1 = 1m
# turn right 90deg
# drive forward until blocked
# turn right 90 deg
# drive forward x2 = 0.5m
# turn left 90deg
# drive forward until blocked
# turn left 90 deg
# drive forward x3 = 0.5m
# turn right 90 deg
# drive forward 0.5m

drive_distance(4000, 0)
piwars2024.imu_turn(-90)
time.sleep(3.0)
piwars2024.set_speed(0, 0)
piwars2024.set_mode(1)
drive_until_blocked()

piwars2024.imu_turn(-180)
time.sleep(3.0)
piwars2024.set_speed(0, 0)
piwars2024.set_mode(1)
drive_distance(2500, -180)

piwars2024.imu_turn(-90)
time.sleep(3.0)
piwars2024.set_speed(0, 0)
piwars2024.set_mode(1)
drive_until_blocked()

piwars2024.imu_turn(0)
time.sleep(3.0)
piwars2024.set_speed(0, 0)
piwars2024.set_mode(1)
drive_distance(2500, 0)

piwars2024.imu_turn(-90)
time.sleep(3.0)
piwars2024.set_speed(0, 0)
piwars2024.set_mode(1)
drive_distance(6500, -90)

#running = True
#while running:
#    while jr.events_available() != 0:
#        event = jr.get_event()
#        if event["value"] == -32767 and event["action"] == 2 and event["button"] == 7:
#            print("pressed up")
#            turn(90)

piwars2024.set_speed(0, 0)
time.sleep(0.5)
piwars2024.set_motor(0, 0, 0, 0)
time.sleep(0.5)
piwars2024.set_mode(2)
time.sleep(0.5)
piwars2024.set_motor(0, 0, 0, 0)
time.sleep(2)

exit(0)

while running:
    im = picam2.capture_array()
    #hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    im = cv2.flip(im, -1)
    cv2.imshow('image', im)

    image_process(im)

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

