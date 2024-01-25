#!/usr/bin/env python3

from adafruit_motorkit import MotorKit

import numpy as np
import cv2

laptop_cam = 6
realsense_cam = 2

#cap = cv2.VideoCapture(4)
cap = cv2.VideoCapture(0)

green_lo = 55
green_hi = 80
blue_lo =  96
blue_hi = 102
red_lo = 0
red_hi = 20

kit = MotorKit()

def boxPos(img, h_lo, h_hi):
    lo = (h_lo, 50, 50)
    hi = (h_hi, 200, 200)

    mask = cv2.inRange(img, lo, hi)
    
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

count = 0
while(True):
    count += 1
    
    # Capture frame-by-frame
    ret, img = cap.read()

    img = img[img.shape[0]//2:img.shape[0]]

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    height, width, _ = img.shape

    if count % 20:
        x, y = boxPos(img_hsv, red_lo, red_hi)
        cv2.circle(img, (x, y), 5, (255, 255, 255), -1)
        cv2.putText(img, "Red", (x - 25, y - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        redx = x

        x, y = boxPos(img_hsv, green_lo, green_hi)
        cv2.circle(img, (x, y), 5, (255, 255, 255), -1)
        cv2.putText(img, "Green", (x - 25, y - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        greenx = x

        x, y = boxPos(img_hsv, blue_lo, blue_hi)
        cv2.circle(img, (x, y), 5, (255, 255, 255), -1)
        cv2.putText(img, "Blue", (x - 25, y - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        bluex = x

        if greenx < width/2 - 50:
            kit.motor1.throttle = 0.5
            kit.motor2.throttle = 0
        elif greenx > width/2 + 50:
            kit.motor1.throttle = 0
            kit.motor2.throttle = 0.5
        else:
            kit.motor1.throttle = None
            kit.motor2.throttle = None

       
    if count % 20:
        h = (int(img_hsv[height//2,width//2][0]) + int(img_hsv[height//2,(width//2)+1][0]) + int(img_hsv[height//2,(width//2)-1][0]))//3
        s = (int(img_hsv[height//2,width//2][1]) + int(img_hsv[height//2,(width//2)+1][1]) + int(img_hsv[height//2,(width//2)-1][1]))//3
        v = (int(img_hsv[height//2,width//2][2]) + int(img_hsv[height//2,(width//2)+1][2]) + int(img_hsv[height//2,(width//2)-1][2]))//3
        #h = img_hsv[height//2,width//2][0`]
 
        print("picker: {}, {}, {}".format(h, s, v))

#    gray = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)
    gray = cv2.inRange(gray, 0, 50)
    gray = np.uint8(gray)

    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    low_threshold = 50
    high_threshold = 150
    edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

    rho = 1  # distance resolution in pixels of the Hough grid
    theta = np.pi / 180  # angular resolution in radians of the Hough grid
    threshold = 15  # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 150  # minimum number of pixels making up a line
    max_line_gap = 20  # maximum gap in pixels between connectable line segments
    line_image = np.copy(img) * 0  # creating a blank to draw lines on

    # Run Hough on edge detected image
    # Output "lines" is an array containing endpoints of detected line segments
    lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                            min_line_length, max_line_gap)

    uppers = []
    downers = []
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                if y1 < y2:
                    uppers.append( [x1, y1, x2, y2] )
                    cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 1)
                else:
                    downers.append( [x1, y1, x2, y2] )
                    cv2.line(line_image, (x1, y1), (x2, y2), (255, 255, 0), 1)

        for line1 in uppers:
            for line2 in downers:

                x1, y1, x2, y2 = line1
                x3, y3, x4, y4 = line2

                denominator = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
#                print(denominator)
                if abs(denominator) > 0.001:
                    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator
                    u = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denominator
#                    print("t={}".format(t))
#                    print("u={}".format(t))
                    inter = False
                    if t >= 0 and t <= 1.0:
                        Px = x1 + t*(x2-x1)
                        Py = y1 + t*(y2-y1)
                        inter = True
                    elif u >= 0 and u <= 1.0:
                        Px = x3 + u*(x4-x3)
                        Py = y3 + u*(y4-y3)
                        inter = True

                    if inter:
                        if Px > 0 and Px < width and Py > 0 and Py < height:
                            center_coordinates = (int(Px), int(Py))
                            radius = 5
                            color = (0, 255, 255)
                            thickness = 2
                            line_image = cv2.circle(line_image, center_coordinates, radius, color, thickness)
#                            print("{}, {}".format(int(Px), int(Py)))

    # Draw the lines on the  image
    lines_edges = cv2.addWeighted(img, 0.8, line_image, 1, 0)

    # Display the resulting frame
 
    center_coordinates = (int(width/2), int(height/2))
    radius = 5
    color = (255, 255, 255)
    thickness = 1
    lines_edges = cv2.circle(lines_edges, center_coordinates, radius, color, thickness)
    
    #cv2.imshow('frame', gray)
    #cv2.imshow('result',lines_edges)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break

# When everything done, release the capture
cap.release()
#cv2.destroyAllWindows()
