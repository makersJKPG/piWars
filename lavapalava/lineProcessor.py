import cv2
import piwars2024
from pid import *
import numpy as np
import time

class lineProcessor:

    def  __init__(self, params=None):

        size = 300, 300, 3
        m = np.zeros(size, dtype=np.uint8) 
        cv2.imshow("image", m)
        size = 300, 300
        m = np.zeros(size, dtype=np.uint8)
        cv2.imshow("mask", m)

        cv2.moveWindow("image", 0, 0)
        cv2.moveWindow("mask", 0, 280)

        limMin = -1.0
        limMax = 1.0
        limMinInt = -1.0
        limMaxInt = 1.0
        T = 0.02
        tau = 1.0

        self.pid = pid(0.01, 0.1, 0.01, T, tau, limMin, limMax, limMinInt, limMaxInt)

        if params is not None:
            self.params = params
        else:
            self.params = {
                "disable_motors": False,
                "roixmin": 0,
                "roixmax": 100,
                "roiymin": 0,
                "roiymax": 100,
                "scale_percent": 100,
                "line_detect_threshold": 100
            }

#        self.motor = motorControl(disable_motors=self.params["disable_motors"])
        piwars2024.set_mode(1)

        self.route_section = 0
        self.route_timer = None
        self.prev_routes = None

        self.tour = [ "right", "left" ]
        self.start_time = time.time()

    def get_roi(self, img):

        imgh, imgw = img.shape

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

    def process3(self, img):

        print("process3()")

        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        img = self.get_roi(img)

        height, width = img.shape

        self.frameHeight = height
        self.frameWidth = width

        side_band_width = 0
        band_height = 40
        img = cv2.rectangle(img, 
                (side_band_width, band_height), 
                (width-side_band_width-1, height-1), 255, -1)

        ret, mask = cv2.threshold(img, 80, 255, cv2.THRESH_BINARY_INV)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        routes = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.params["line_detect_threshold"]:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX = 0
                    cY = 0
                routes.append({
                    "cX": cX,
                    "cY": cY,
                    "area": area
                })
                print("Contour: A={}, x,y={},{}".format(area, cX, cY))
                cv2.circle(img, (cX, cY), 5, 127, 1)

        # select route
        numroutes = len(routes)
        print("Routes available: {}".format(numroutes))

        if self.start_time is not None and time.time() - self.start_time > 10:
            print("Start time passed")
            self.start_time = None

        if self.start_time is None:
            if self.route_timer is not None and time.time() - self.route_timer > 1:
                if self.route_section < len(self.tour)-1:
                    self.route_section += 1
                    print("Passed junction {} on route".format(self.route_section))
                else:
                    print("Passed junction after tour ended")
 
                self.route_timer = None
                print("Section is {}".format(self.route_section))
                print("Bear {}".format(self.tour[self.route_section]))

        if numroutes == 0:
            # no routes available
            route = None

        elif numroutes == 1:

            #if self.junction_passed < len(self.tour) - 1:
            #    self.junction_passed += 1
            #    print("Junction passed: {}".format(self.junction_passed))
            
            # follow the only route available
            route = routes[0]
            
        else:
            # here we have multiple routes to choose from
            
            if self.start_time is None:
                if self.route_timer is None:
                    print("Entering junction - bear {}".format(self.tour[self.route_section]))
                    self.route_timer = time.time()

            max_x = routes[0]["cX"]
            for r in routes:
                # find leftmost or rightmost route
                if self.tour[self.route_section] == "left":
                    if r["cX"] <= max_x:
                        max_x = r["cX"]
                        route = r
                else:
                    if r["cX"] >= max_x:
                        max_x = r["cX"]
                        route = r

        if route is not None:
            # set direction of travel

            measurement = route["cX"] - self.frameWidth/2
            setpoint = 0

            output = self.pid.update(setpoint, measurement)
            adj_r = -output if output < 0 else 0
            adj_l = output if output > 0 else 0
            print("measurement={}, output={}".format(measurement, output))
            r_speed = int(55 * (1.0 - adj_r))
            l_speed = int(55 * (1.0 - adj_l))
            piwars2024.set_speed(l_speed, r_speed)
        
            #self.prev_routes = len(routes)
        else:
            pass
            piwars2024.set_speed(0, 0)

        cv2.imshow("mask", mask)
        cv2.imshow("image", img)

    def process2(self, img):

        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        img = self.get_roi(img)

        height, width = img.shape

        self.frameHeight = height
        self.frameWidth = width

        ret, mask = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY_INV)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        routes = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.params["line_detect_threshold"]:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX = 0
                    cY = 0
                routes.append({
                    "cX": cX,
                    "cY": cY,
                    "area": area
                })
                #print("Contour: A={}, x,y={},{}".format(area, cX, cY))
                cv2.circle(img, (cX, cY), 5, 127, 1)

        # select route
        if len(routes) > 0:
            #print("Routes available: {}".format(len(routes)))

            if self.prev_routes is not None:
                if len(routes) == 1 and self.prev_routes > 1:
                    if self.junction_passed < len(self.tour) - 1:
                        self.junction_passed += 1
                        print("Junction passed: {}".format(self.junction_passed))
                    
                max_x = routes[0]["cX"]
                for r in routes:
                    if self.tour[self.junction_passed] == "left":
                        if r["cX"] <= max_x:
                            max_x = r["cX"]
                            route = r
                    else:
                        if r["cX"] >= max_x:
                            max_x = r["cX"]
                            route = r
 
                measurement = route["cX"] - self.frameWidth/2
                setpoint = 0

                output = self.pid.update(setpoint, measurement)
                adj_r = -output if output < 0 else 0
                adj_l = output if output > 0 else 0
                #print("measurement={}, output={}".format(measurement, output))
                ## self.motor.throttle(1.0 - adj_l, 1.0 - adj_r)
        
            self.prev_routes = len(routes)
        else:
            pass
            ## self.motor.throttle(0, 0)

        cv2.imshow("mask", mask)
        cv2.imshow("image", img)

        key = cv2.waitKey(1)
        if key == 'q':
            print("Q")
            return True
        else:
            return False



    def process(self, img):

        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        img = self.get_roi(img)

        height, width = img.shape

        self.frameHeight = height
        self.frameWidth = width

        ret, mask = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY_INV)
        self.line = self.boxPos(mask) if cv2.countNonZero(mask) > self.params["line_detect_threshold"] else None

        if self.line is not None:
            cv2.circle(img, self.line, 5, 127, 1)

        cv2.imshow("mask", mask)
        cv2.imshow("image", img)

        measurement = self.get_measurement(mask)
        setpoint = 0

        if measurement is not None:
            output = self.pid.update(setpoint, measurement)
            adj_r = -output if output < 0 else 0
            adj_l = output if output > 0 else 0
            print("measurement={}, output={}".format(measurement, output))
            ## self.motor.throttle(1.0 - adj_l, 1.0 - adj_r)
        else:
            pass
            ## self.motor.throttle(0, 0)

        key = cv2.waitKey(1)
        if key == 'q':
            print("Q")
            return True
        else:
            return False

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

    def get_measurement(self, img):

        if self.line is not None:
            return self.line[0] - self.frameWidth/2
        else:
            return None
