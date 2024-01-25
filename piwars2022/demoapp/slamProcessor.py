import cv2
from cube import *
from motorControl import *
from pid import *
import numpy as np
import math

class slamProcessor:

    def  __init__(self, params=None):

        self.motor = motorControl()
        limMin = -1.0
        limMax = 1.0
        limMinInt = -1.0
        limMaxInt = 1.0
        T = 0.02
        tau = 1.0

        self.pid = pid(0.005, 0.05, 0.01, T, tau, limMin, limMax, limMinInt, limMaxInt)
        self.cube = cube()

        if params is not None:
            self.params = params
        else:
            self.params = {
                "roixmin": 0,
                "roixmax": 100,
                "roiymin": 0,
                "roiymax": 100,
                "scale_percent": 100,
                "dot_max_area": 100
            }

        # order tl, tr, br, bl 
        source_pts = np.array([
            [112.0, 345.0], 
            [546.0, 347.0], 
            [612.0, 175.0], 
            [38.0, 179.0]
        ], dtype="float32")
        dest_pts = np.array([
            [-50.0, 150.0],
            [50.0, 150.0],
            [50.0, 100.0],
            [-50.0, 100.0]
        ], dtype="float32")
        self.H = cv2.getPerspectiveTransform(source_pts, dest_pts)

        self.frameCount = 0

    def get_roi(self, img):

        imgh, imgw, _ = img.shape

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

        self.frameCount += 1

        # get roi...
        img = self.get_roi(img)
        height, width, _ = img.shape
        self.frameHeight = height
        self.frameWidth = width

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, mask = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=1)
        mask = cv2.erode(mask, kernel, iterations=1)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # real world map 200x200 pixels...
        rwmap = np.zeros((200, 200), np.uint8)

        # plot camera viewport on real world map...
        tl = np.dot(self.H, [0.0, height-1.0, 1.0])
        tr = np.dot(self.H, [width-1.0, height-1.0, 1.0])
        br = np.dot(self.H, [width-1.0, 0.0, 1.0])
        bl = np.dot(self.H, [0.0, 0.0, 1.0])
        m = 100.0
        cv2.line(rwmap, (int(tl[0]/tl[2]+m),int(tl[1]/tl[2])), (int(tr[0]/tr[2]+m),int(tr[1]/tr[2])), 255, 1)
        cv2.line(rwmap, (int(tr[0]/tr[2]+m),int(tr[1]/tr[2])), (int(br[0]/br[2]+m),int(br[1]/br[2])), 255, 1)
        cv2.line(rwmap, (int(br[0]/br[2]+m),int(br[1]/br[2])), (int(bl[0]/bl[2]+m),int(bl[1]/bl[2])), 255, 1)
        cv2.line(rwmap, (int(bl[0]/bl[2]+m),int(bl[1]/bl[2])), (int(tl[0]/tl[2]+m),int(tl[1]/tl[2])), 255, 1)

        # plot target on real world map...
        targetx = 0
        targety = 10

        dots = []
        rwdots = []
        if len(contours) < 20:
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 0.0 and area < self.params["dot_max_area"]:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                    else:
                        cX = 0
                        cY = 0

                    # dot coordinate in camera viewport...
                    dot = [cX, cY, 1.0]

                    # convert viewport coordinate to real world coordinate...
                    rwdot = np.dot(self.H, dot) 
                    rwdot[0] = rwdot[0]/rwdot[2]
                    rwdot[1] = rwdot[1]/rwdot[2]

                    # save dots for the future...
                    dots.append(dot)
                    rwdots.append(rwdot)

                    # plot dot on camera viewport...
                    cv2.circle(img, (cX, cY), 5, (255, 255, 0), 1)

            # open gripper if there is one dot in view...
            if self.frameCount % 20 == 0:
                if len(rwdots) == 1:
                    self.motor.open_gripper()
                else:
                    self.motor.close_gripper()

            # calculate features for dots...
#           dotmeta = get_dotmeta(rwdots)

            if len(rwdots) == 1:
                rwdotx = rwdots[0][0]
                rwdoty = rwdots[0][1]


                print(rwdot)


        else:
            print("dot overload")

        img_hue = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        green = self.cube.getGreenPos(img_hue)
        print("Green: {}".format(green))
 
        red = self.cube.getRedPos(img_hue)
        print("Red: {}".format(red))
 
        blue = self.cube.getBluePos(img_hue)
        print("Blue: {}".format(blue))

        if green is not None:
            cv2.circle(img, (green[0], green[1]), 5, (255, 255, 0), 1)

        if red is not None:
            cv2.circle(img, (red[0], red[1]), 5, (255, 255, 0), 1)

        if blue is not None:
            cv2.circle(img, (blue[0], blue[1]), 5, (255, 255, 0), 1)

        mask = cv2.flip(mask, 0)
        img = cv2.flip(img, 0)
        rwmap = cv2.flip(rwmap, 0)
        rwmap = cv2.resize(rwmap, (rwmap.shape[1]*2, rwmap.shape[0]*2))
#        cv2.imshow("mask", mask)
        cv2.imshow("image", img)
        cv2.imshow("world", rwmap)

        key = cv2.waitKey(1)
        if key == 'q':
            print("Q")
            return True
        else:
            return False

    def got_dotmeta(self, rwdots):
        dotmeta = [[] for j in range(len(rwdots))]

        if len(rwdots) > 0:
            for i, dot in enumerate(rwdots):
                x1 = dot[0]
                y1 = dot[1]
                cv2.circle(rwmap, (int(x1 + rwmap.shape[1]/2), int(y1)), 5, 255, 1)
                for j in range(i+1, len(rwdots)):
                    x2 = rwdots[j][0]
                    y2 = rwdots[j][1]
                    dist = int(math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)))
                    dotmeta[i].append(dist)
                    dotmeta[j].append(dist)

            for meta in dotmeta:
                meta.sort()

            if self.frameCount % 40 == 0:
                print("---")
                for i, meta in enumerate(dotmeta):
                    print("{} - {}".format(i, meta))

            #print(dotmeta)

        return dotmeta

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

    # get camera perspective transform matrix from src and dst points

    # plot guide points on arena map
    # maintain list of guide points

    # find pose by matching guide points list and observed points

    # process:
    #   get observed viewport guide points from viewport image
    #   transform observed viewpoint guide points to observed arena guide points using camera perspective transform matrix
    #   get pose matrix by matching observed arena guide points with known arena guide points
    #   if there exists observed points that do not exist in known guide points then
    #       add the missing observed points to known guide points 

    def order_points(pts):

        # initialzie a list of coordinates that will be ordered such that the first entry in the list is the top-left,
        # the second entry is the top-right, the third is the bottom-right, and the fourth is the bottom-left
        rect = np.zeros((4, 2), dtype = "float32")

        # the top-left point will have the smallest sum, whereas the bottom-right point will have the largest sum
        s = pts.sum(axis = 1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]

        # now, compute the difference between the points, the top-right point will have the smallest difference,
        # whereas the bottom-left will have the largest difference
        diff = np.diff(pts, axis = 1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]

        # return the ordered coordinates
        return rect

    def four_point_transform(image, pts):
        # obtain a consistent order of the points and unpack them individually
        rect = order_points(pts)
        (tl, tr, br, bl) = rect

        # compute the width of the new image, which will be the maximum distance between bottom-right and bottom-left
        # x-coordiates or the top-right and top-left x-coordinates
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))

        # compute the height of the new image, which will be the
        # maximum distance between the top-right and bottom-right
        # y-coordinates or the top-left and bottom-left y-coordinates
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))

        # now that we have the dimensions of the new image, construct
        # the set of destination points to obtain a "birds eye view",
        # (i.e. top-down view) of the image, again specifying points
        # in the top-left, top-right, bottom-right, and bottom-left
        # order
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype = "float32")
        # compute the perspective transform matrix and then apply it
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
        # return the warped image
        return warped
