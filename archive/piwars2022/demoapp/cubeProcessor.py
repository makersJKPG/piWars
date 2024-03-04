from motorControl import *
from pid import *
import cv2
from cubeUtil import *
import numpy as np
import time

class cubeProcessor:

    def  __init__(self, params = None):

        size = 300, 300, 3
        m = np.zeros(size, dtype=np.uint8) 
        cv2.imshow("bgr", m)
        size = 300, 300
        m = np.zeros(size, dtype=np.uint8)
        cv2.imshow("mask", m)

        cv2.moveWindow("bgr", 0, 0)
        cv2.moveWindow("mask", 0, 580)

        # time taken by last call to process method
        self.process_time = 0

        # will be set to true when requesting shutdown
        self.shutdown = False

        # used to measure time since last time gripper was set
        # servo will be released when counter reaches zero
        self.gripperCount = 0

        if params is not None:
            self.params = params
        else:
            self.params = {
                "enable_colorpicker": True,
                "disable_motors": False,
                "show_timing": False,
                "roixmin": 0,
                "roixmax": 100,
                "roiymin": 5,
                "roiymax": 100,
                "scale_percent": 50
            }

        self.motor = motorControl(disable_motors=self.params["disable_motors"])

        # available tasks:
        #  grab $spin_direction $color
        #  deliver $spin_direction $color
        #  start task
        #  end task

       # sequence of tasks to do
        self.tasks = [
            "drive 40 -1 -1",
            "gripper open",
            
            "grab left red",
            "gripper close",
            "wait 6",
            "drive 30 -1 -1",

            "deliver left yellow 0.20",
            "drive 13 1 1",
            "gripper open",
            "wait 6",
            "drive 25 -1 -1",
            
            "grab right green",
            "gripper close",
            "wait 6",
            "drive 45 -1 -1",
            
            "deliver left red -0.25",
            "drive 15 1 1",
            "gripper open",
            "wait 6",
            "drive 25 -1 -1",

            "grab right blue",
            "gripper close",
            "wait 6",
            "drive 55 -1 -1",

            "deliver left green -0.25",
            "drive 15 1 1",
            "gripper open",
            "wait 6",
            "drive 40 -1 -1",
            "gripper close"
        ]

        # sequence of tasks to do
        self.tasks = [
            # reverse, then open gripper
            "drive 40 -1 -1",
            "gripper open",
            
            # grab red cube, then reverse
            "visit left red 0.35",
            "gripper close",
            "wait 3",
            "drive 30 -1 -1",

            # deliver red cube to yellow area
            "visit left yellow 0.6 0.2",
            "drive 13 1 1",
            "gripper open",
            "wait 3",
            "drive 25 -1 -1",
            
            # grab green cube
            "visit right green 0.35",
            "gripper close",
            "wait 3",
            "drive 45 -1 -1",
            
            # deliver green cube to right of red cube
            "visit left red 0.6 -0.3",
            "drive 13 1 1",
            "gripper open",
            "wait 3",
            "drive 60 -1 -1",

            # grab blue cube
            "visit right blue 0.35",
            "gripper close",
            "wait 3",
            "drive 45 -1 -1",

            # deliver blue cube to right of green cube
            "visit left green 0.6 -0.3",
            "drive 13 1 1",
            "gripper open",
            "wait 3",
            "drive 40 -1 -1",
            "gripper close"
        ]

        # set of integer representing frames processed while in each task
        self.task_counters = [0] * len(self.tasks)

        self.startMission()

        # a task will set objectiveMet when objective has been met
        self.objectiveMet = False

        # missonComplete will be set to true when there no more tasks to do
        self.missionComplete = False

        # boolean toggles when user presses 'p'
        self.pause = False

    def startMission(self):
        self.setTask(0)

    def nextTask(self):
        if self.task < len(self.tasks) - 1:
            print("=======> Switching to next task")
            self.setTask(self.task + 1)
            return True
        else:
            print("Mission completed")
            self.missionComplete = True
            return False

    def setTask(self, task):
        self.task = task
        print("Task is: {}".format(self.tasks[self.task]))

    def currentTask(self):
        return self.tasks[self.task]

    def currentTaskCommand(self):
        return self.tasks[self.task].split()[0]

    def taskParameter(self, param):
        return self.getTaskParameters[param]

    def getTaskParameters(self):
        return self.tasks[self.task].split()

    def taskCounter(self):
        return self.task_counters[self.task]

    def updateTaskCounter(self):
        self.task_counters[self.task] += 1

    def get_roi(self, img):
        imgh = img.shape[0]
        imgw = img.shape[1]

        # crop image...
        roixmin = int(imgw * self.params["roixmin"] / 100.0)
        roixmax = int(imgw * self.params["roixmax"] / 100.0)
        roiymin = int(imgh * self.params["roiymin"] / 100.0)
        roiymax = int(imgh * self.params["roiymax"] / 100.0)
        roi = img[roiymin:roiymax,roixmin:roixmax]

        # scale image...
        width = int(roi.shape[1] * self.params["scale_percent"] / 100.0)
        height = int(roi.shape[0] * self.params["scale_percent"] / 100.0)
        roi = cv2.resize(roi, (width, height))

#        cv2.rectangle(img, (roixmin, roiymin), (roixmax, roiymax), (150, 150, 0), 2)

        return roi, img

    def maskImg(self, img):

        height = img.shape[0]
        width = img.shape[1]

        pt1 = (width/2, 0)
        pt2 = (0, 0)
        pt3 = (0, height/2)
        triangle_contour = np.array( [pt1, pt2, pt3] )
        cv2.drawContours(img, [triangle_contour], 0, (0,0,0), -1)

        pt1 = (width/2, 0)
        pt2 = (width-1, height/2)
        pt3 = (width-1, 0)
        triangle_contour = np.array( [pt1, pt2, pt3] )
        cv2.drawContours(img, [triangle_contour], 0, (0,0,0), -1)

#        cv2.imshow("image", img)
        return img

    def process(self, img):

        start_time = time.time()

        if self.gripperCount > 0:
            self.gripperCount -= 1
            if self.gripperCount == 0:
                print("release servo")
                self.motor.release_servo()

        if self.missionComplete == True:
            return

        self.img, self.orig_img = self.get_roi(img)
#        self.img = self.maskImg(img)

        height, width, _ = self.img.shape
        self.frameHeight = height
        self.frameWidth = width

        self.hsv = cv2.cvtColor(self.img, cv2.COLOR_RGB2HSV)
        self.bgr = self.img #cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)
        
        if self.params["enable_colorpicker"]:
            self.colorpicker = self.hsv[self.frameHeight//2][self.frameWidth//2]
            cv2.circle(self.bgr, (self.frameWidth//2, self.frameHeight//2), 5, (0, 255, 255), 1)
            #print("hsv = {}".format(self.colorpicker))
            org = (20, self.frameHeight-50)
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 0.5
            color = (255, 255, 255)
            thickness = 1
            self.bgr = cv2.putText(self.bgr, "hsv = {}".format(self.colorpicker), org, font, 
                   fontScale, color, thickness, cv2.LINE_AA)

        self.objectiveMet = False

        task_params = self.getTaskParameters()
        task_method = getattr(self, task_params[0] + "Task")
        task_method(task_params)

        self.updateTaskCounter()

        if self.objectiveMet:
            if self.nextTask() == False:
                print("shutting down")
                self.motor.release()
                self.motor.release_servo()
                self.shutdown = True

        end_time = time.time()
        self.process_time = end_time - start_time
        if self.params["show_timing"]:
            print("Time taken: {}".format(self.process_time))
 
        cv2.imshow("bgr", self.bgr)
        key = cv2.waitKey(1)
        if key == 'q':
            self.shutdown = True
            print("Q")
            return True
        elif key == 'p':
            self.pause = not self.pause
            print("Pause")
        else:
            return False

    def driveTask(self, params):
        num = int(params[1])
        throttle_left = float(params[2])
        throttle_right = float(params[3])

        counter = self.taskCounter()

        if counter == 0:
            self.motor.throttle(throttle_left, throttle_right)

        if counter == num:
            self.motor.release()
            self.objectiveMet = True

    def waitTask(self, params):
        print("waitTask {}".format(self.taskCounter()))

        if int(params[1]) == self.taskCounter():
            self.objectiveMet = True
  
    def visitTask(self, params):

        if len(params) != 4 and len(params) != 5:
            print("Incorrect number of params for visit command. params={}".format(params))
            return

        color = params[2]
        cube = cubeUtil.getCubePos(self.hsv, color, roi=75)

        direction = params[1]
        if direction == "left":
            direction = -1
        elif direction == "right":
            direction = 1
        else:
            print("Invalid direction, must be right or left")

        target_y = float(params[3])

        if len(params) == 5:
            setpoint_target_x = float(params[4])
            print("setpoint targetx by {}".format(setpoint_target_x))
        else:
            setpoint_target_x = 0

        counter = self.taskCounter()
        if counter == 0:
            print("Setting up new PID")
            limMin = -1.0
            limMax = 1.0
            limMinInt = -1.0
            limMaxInt = 1.0
            T = 0.2
            tau = 1.0
            self.pid = pid(3.0, 1.0, 0.01, T, tau, limMin, limMax, limMinInt, limMaxInt)

        if cube is not None:
            cv2.circle(self.bgr, cube, 5, (255, 255, 255), -1)

            measurementx = (cube[0] - self.frameWidth/2) / self.frameWidth
            measurementy = (self.frameHeight - cube[1]) / self.frameHeight

            #target_y = 0.6
            if measurementy > target_y:
                setpoint = setpoint_target_x
                output = self.pid.update(setpoint, measurementx)
                print("measurement = x:{:.2f}, y:{:.2f}, target_y={}, output={:.2f}".format(measurementx, measurementy, target_y, output))

                if output > 0:
                    r_adj = output
                    l_adj = 0
                if output < 0:
                    r_adj = 0
                    l_adj = -output
                self.motor.throttle(1.0 - l_adj, 1.0 - r_adj)
            else:
                # objective met, stop motors
                self.objectiveMet = True
                print("Objective met")
                self.motor.release()
        else:
            # seek cube by spinning round...
            self.motor.throttle(direction * -1, direction * 1)


    def deliverTask(self, params):
        color = params[2]
        cube = cubeUtil.getCubePos(self.hsv, color, roi=75)

        direction = params[1]
        if direction == "left":
            direction = -1
        elif direction == "right":
            direction = 1
        else:
            print("Invalid direction, must be right or left")

        if len(params) == 4:
            setpoint_targetx = float(params[3])
            print("setpoint targetx by {}".format(setpoint_targetx))
        else:
            setpoint_targetx = 0

        counter = self.taskCounter()
        if counter == 0:
            print("Setting up new PID")
            limMin = -1.0
            limMax = 1.0
            limMinInt = -1.0
            limMaxInt = 1.0
            T = 0.2
            tau = 1.0
            self.pid = pid(3.0, 1.0, 0.01, T, tau, limMin, limMax, limMinInt, limMaxInt)

        if cube is not None:
            cv2.circle(self.bgr, cube, 5, (255, 255, 255), -1)

            measurementx = (cube[0] - self.frameWidth/2) / self.frameWidth
            measurementy = (self.frameHeight - cube[1]) / self.frameHeight

            # y ~ 5000 at top of screen
            # y ~ 0 at bottom of screen

            target_y = 0.6
            if measurementy > target_y:
                setpoint = setpoint_targetx
                output = self.pid.update(setpoint, measurementx)
                print("measurement = x:{:.2f}, y:{:.2f}, target_y={}, output={:.2f}".format(measurementx, measurementy, target_y, output))

                if output > 0:
                    r_adj = output
                    l_adj = 0
                if output < 0:
                    r_adj = 0
                    l_adj = -output
                self.motor.throttle(1.0 - l_adj, 1.0 - r_adj)
            else:
                # objective met, stop motors
                self.objectiveMet = True
                print("Objective met")
                self.motor.release()
        else:
            # seek cube by spinning round...
            self.motor.throttle(direction * -1, direction * 1)

    def grabTask(self, params):
        #print("grabTask {}".format(self.taskCounter()))

        color = params[2]
        cube = cubeUtil.getCubePos(self.hsv, color)

        direction = params[1]
        if direction == "left":
            direction = -1
        elif direction == "right":
            direction = 1
        else:
            print("Invalid direction, must be right or left")

        counter = self.taskCounter()
        if counter == 0:
            print("Setting up new PID")
            limMin = -1.0
            limMax = 1.0
            limMinInt = -1.0
            limMaxInt = 1.0
            T = 0.2
            tau = 1.0
            self.pid = pid(3.0, 1.0, 0.01, T, tau, limMin, limMax, limMinInt, limMaxInt)
            
        if cube is not None:
            cv2.circle(self.bgr, cube, 5, (255, 255, 255), -1)

            measurementx = (cube[0] - self.frameWidth/2) / self.frameWidth
            measurementy = (self.frameHeight - cube[1]) / self.frameHeight

            # y ~ 5000 at top of screen
            # y ~ 0 at bottom of screen

            target_y = 0.16
#            target_y = 0
            if measurementy > target_y:
                setpoint = 0
                output = self.pid.update(setpoint, measurementx)
                print("measurement = x:{:.2f}, y:{:.2f}, target_y={}, output={:.2f}".format(measurementx, measurementy, target_y, output))

                if output > 0:
                    r_adj = output
                    l_adj = 0
                if output < 0:
                    r_adj = 0
                    l_adj = -output
                self.motor.throttle(1.0 - l_adj, 1.0 - r_adj)
            else:
                # objective met, stop motors
                self.objectiveMet = True
                print("Objective met")
                self.motor.release()
        else:
            # seek cube by spinning round...
            self.motor.throttle(direction * -1, direction * 1)

    def gripperTask(self, params):
        if params[1] == "open":
            self.open_gripper()
        elif params[1] == "close":
            self.close_gripper()
        else:
            print("Error: unknown gripper parameter - {}".format(params[1]))
        self.objectiveMet = True

    def open_gripper(self):
        print("open gripper")
        self.motor.open_gripper()
        self.gripperCount = 5

    def close_gripper(self):
        print("close gripper")
        self.motor.close_gripper()
        self.gripperCount = 5


