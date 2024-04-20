from threading import Thread as t
from cv2 import (
    cvtColor,putText,morphologyEx,findContours,contourArea,
    boundingRect,inRange,bitwise_or,#VideoWriter_fourcc,VideoCapture,
    #CAP_DSHOW,CAP_PROP_FRAME_WIDTH,CAP_PROP_FRAME_HEIGHT,
    #CAP_PROP_FPS,CAP_PROP_FOURCC,
    FONT_HERSHEY_COMPLEX,COLOR_RGB2BGR,MORPH_OPEN,
    MORPH_CLOSE,CHAIN_APPROX_SIMPLE,RETR_EXTERNAL,COLOR_BGR2HSV)
from numpy import array,array_equal,uint8,ones,zeros,asarray,int64
from math import sqrt
from libcamera import Transform
#import tensorflow as tf

import time

DEBUGGING=True
if DEBUGGING:
    import cv2 as cv

class Cam:
    from picamera2 import Picamera2

    def __init__(
            self,
            #camNum=0,
            capFormat='RGB888',
            capWidth=int(640),
            capHeight=int(480),
            #fps=30,
            #codec=VideoWriter_fourcc(*'MJPG')
            ):
        #self.cap=VideoCapture(camNum,CAP_DSHOW)
        self.height=capHeight
        self.width=capWidth
        self.cap=self.Picamera2()
        
        #Find fastest camera config
 #       fps=0
 #       capFormat=None
 #       for modes in self.cap.sensor_modes:
 #           if (capWidth,capHeight)==modes['size'] and modes['fps']>fps:
 #               fps=modes['fps']
 #               capFormat=modes['format']
 #       print('fps=',fps,'format=',capFormat)

        self.cap.configure(self.cap.create_preview_configuration(main={"format":capFormat,"size": (capWidth,capHeight)},transform=Transform(hflip=1, vflip=1)))
        self.cap.start()
        self.longestPossibleOffset=sqrt(pow(capWidth*.5,2)+pow(capHeight*.5,2))
        self.globalCenter=(int(capWidth*.5),int(capHeight*.5))
        #self.cap.set(CAP_PROP_FRAME_WIDTH,capWidth)
        #self.cap.set(CAP_PROP_FRAME_HEIGHT,capHeight)
        #self.cap.set(CAP_PROP_FPS,fps)
        #self.cap.set(CAP_PROP_FOURCC,codec)
        self.frame=[]
        self.greenMask=array([])
        self.greenCenter=((0,0),0,0,(0,0),(0,0))
        self.redMask=array([])
        self.redCenter=((0,0),0,0,(0,0),(0,0))
        self.blueMask=array([])
        self.blueCenter=((0,0),0,0,(0,0),(0,0))
        self.yellowMask=array([])
        self.yellowCenter=((0,0),0,0,(0,0),(0,0))
        self.kernel=ones((5,5),uint8)
        self.readSuccess=False
        self.createMasks=False
        self.emptyFrame=zeros((capWidth,capHeight),uint8)
        self.inference={}
        putText(
            self.emptyFrame,'Error reading frame',(25,25), 
            FONT_HERSHEY_COMPLEX,1,(255,255,255), 1)
        self.run=True
       # self.inferenceModel=tf.saved_model.load('/home/pi/projects/piWars/ecodisaster/trained_models')
       # self.inferenceThread=t(target=self.runInference,args=(),daemon=True)
       # self.inferenceThread.start()        
#        self.categoryIndex=label_map_util.create_category_index_from_labelmap(
#            'ecodisaster/barrel_inference_graph/labelmap.pbtxt',use_display_name=True)
        self.capThread=t(target=self.captureFrame,args=(),daemon=True)
        # self.cvtThread=t(target=self.createColorMasks,args=(),daemon=True)
        # self.cvtThread.start()
        self.capThread.start()
        #print('Width:\t'+str(self.cap.get(CAP_PROP_FRAME_WIDTH)))
        #print('Height:\t'+str(self.cap.get(CAP_PROP_FRAME_HEIGHT)))


    def setMaskCreation(self,createMask):
        self.createMasks=createMask

  #  def setSize(self,newWidth,newHeight):
  #      self.cap.set(CAP_PROP_FRAME_WIDTH,newWidth)
  #      self.cap.set(CAP_PROP_FRAME_HEIGHT,newHeight)

    def getFrame(self):
        if self.readSuccess:
            return self.frame
        else:
            return cvtColor(self.emptyFrame,COLOR_RGB2BGR)

  #  def getFrameSize(self):
  #      return (self.cap.get(CAP_PROP_FRAME_WIDTH),self.cap.get(CAP_PROP_FRAME_HEIGHT))

    # Can only be used after createMasks have been set to True (using function setMaskCreation)
    def getRedMask(self):
        self.redMask=morphologyEx(src=self.redMask,
                                     op=MORPH_OPEN,
                                     kernel=self.kernel)
        return morphologyEx(self.redMask,MORPH_CLOSE,self.kernel)
    def getRedCenter(self):
        return self.redCenter

    # Can only be used after createMasks have been set to True (using function setMaskCreation)
    def getGreenMask(self):
        self.greenMask=morphologyEx(src=self.greenMask,
                                       op=MORPH_OPEN,
                                       kernel=self.kernel)
        return morphologyEx(self.greenMask,MORPH_CLOSE,self.kernel)
    def getGreenCenter(self):
        return self.greenCenter

    # Can only be used after createMasks have been set to True (using function setMaskCreation)
    def getBlueMask(self):
        self.blueMask=morphologyEx(src=self.blueMask,
                                       op=MORPH_OPEN,
                                       kernel=self.kernel)
        return morphologyEx(self.blueMask,MORPH_CLOSE,self.kernel)
    def getBlueCenter(self):
        return self.blueCenter
    # Can only be used after createMasks have been set to True (using function setMaskCreation)
    def getYellowMask(self):
        self.yellowMask=morphologyEx(src=self.yellowMask,
                                       op=MORPH_OPEN,
                                       kernel=self.kernel)
        return morphologyEx(self.yellowMask,MORPH_CLOSE,self.kernel)
    def getYellowCenter(self):
        return self.yellowCenter

    # def runInference(self):
        # image=self.frame
        # inputTensor=tf.convert_to_tensor(asarray(image))
        # inputTensor=inputTensor[tf.newaxis,...]

        # outputDict=self.inferenceModel(inputTensor)
        # largestArea=0
        # largestAreaCenter=(0,0)
        # x,y,w,h=(0,0,0,0)
        # rw,rh,rx,ry=(0,0,0,0)
        # cla=0
        # offset=0
        # numDetections=int(outputDict.pop('num_detections'))
        # outputDict={key:value[0, :numDetections].numpy()
                    # for key,value in outputDict.items()}
        # outputDict['num_detections']=numDetections
        # outputDict['detection_classes']=outputDict['detection_classes'].astype(int64)
    

#        if 'detection_masks' in outputDict:
#            detectionMasksReframed=util_ops.reframe_box_masks_to_image_masks(
#                outputDict['detecion_masks'],outputDict['detection_boxes'],
#                image.shape[0],
#                image.shape[1])
#            detectionMasksReframed=tf.cast(detectionMasksReframed>.5,tf.uint8)
#            outputDict['detection_masks_refraimed']=detectionMasksReframed.numpy()


        # ~ if ('detection_boxes' in outputDict and 
            # ~ 'detection_scores' in outputDict and 
            # ~ 'detection_classes' in outputDict):
            # ~ for box,cl,score in zip(outputDict['detection_boxes'],outputDict['detection_classes'],outputDict['detection_scores']):
                # ~ if score>.5 and (cl==1 or cl==2):
                    # ~ x,y=((int(box[1]*image.shape[1])),(int(box[0]*image.shape[0])))
                    # ~ w,h=((int(box[3]*image.shape[1])),(int(box[2]*image.shape[0])))
                    # ~ area=(w-x)*(h-y)
                    # ~ cla=cl
                    # ~ if area>largestArea:
                        # ~ rw,rh,rx,ry=(w,h,x,y)
                        # ~ largestArea=area
                        # ~ cx,cy=(int(((w-x)*.5)+x),int(((h-y)*.5)+y))
                        # ~ largestAreaCenter=(cx,cy)
                        # ~ offset=int(sqrt(pow(cx,2)+pow(cy,2))-
                            # ~ sqrt(pow(self.globalCenter[0],2)+pow(self.globalCenter[1],2)))
            # ~ return {
                # ~ 'class':cla,
                # ~ 'largestArea':largestArea,
                # ~ 'largestAreaCenter':largestAreaCenter,
                # ~ 'offset':offset,
                # ~ 'w':rw,
                # ~ 'h':rh,
                # ~ 'x':rx,
                # ~ 'y':ry         
            # ~ }
        # ~ return {}

    #Search for object in mask, calculates center of object using blounding box coordinates.
    #Returns center coordinates of largest object and offset from view (global) center.
    #If no objet is found coordinates and offset will by default be outside of view
    def findCenter(self,mask):
        contours,_=findContours(
            mask,
            RETR_EXTERNAL,       #Find only external details
            CHAIN_APPROX_SIMPLE) #Compress contours to endpoints (x,y,w,h)
        largest_area=0

        #Coordinates for obj bounding box (starts outside the view)
        x,y,rw,rh=(self.width*10,self.height*10,0,0)
        
        if len(contours) > 0:
            #Use contours to parse out closest object in mask
            for c in contours:
                area=contourArea(c)
                if area>largest_area:
                    x,y,rw,rh=boundingRect(c)
                    largest_area=area
        #Calculate return values 
        center=x+int(rw*.5),y+int(rh*.5)
        offset=int(sqrt(pow(center[0],2)+pow(center[1],2))-
                sqrt(pow(self.globalCenter[0],2)+pow(self.globalCenter[1],2)))
        return(
            center,         #Object center point
            offset,         #Offset from global center 
            largest_area,   #Object area
            (rw,rh),        #Objects width and height
            (x,y))          #Starting point of bounding box for object
    
    def createColorMasks(self,frame):
        # while self.run:
        if self.readSuccess:
            hsv=cvtColor(frame,COLOR_BGR2HSV)
            greenMask=inRange(
                hsv,
                array([35,55,55]),
                array([75,255,255]))
            
            blueMask=inRange(
                hsv,
                array([90,0,0]),
                array([125,255,255]))
            
            yellowMask=inRange(
                hsv,
                array([14,128,127]),
                array([35,255,255]))
            
            redMask=bitwise_or(
                inRange(
                    hsv,
                    array([0, 75, 70]),
                    array([10, 255, 255])),
                inRange(
                    hsv,
                    array([170, 75, 70]),
                    array([180, 255, 255])))   
        return (greenMask,blueMask,yellowMask,redMask)

    def captureFrame(self):
        while self.run:
            #self.readSuccess,self.frame=self.cap.read()
            self.frame=self.cap.capture_array()
            if not array_equal(self.frame,[]):
                self.readSuccess=True
                
            if self.createMasks and self.readSuccess:
                #Create Masks for all relevant colors
                (self.greenMask,self.blueMask,self.yellowMask,
                self.redMask)=self.createColorMasks(self.frame)
                
                #Get objet information (center, area, width and height, bounding box)
                self.greenCenter=self.findCenter(self.greenMask)
                self.blueCenter=self.findCenter(self.blueMask)
                self.yellowCenter=self.findCenter(self.yellowMask)
                self.redCenter=self.findCenter(self.redMask)

    def destroy(self):
        self.capThread=0
        # self.cvtThread=0
        self.run=False
        self.cap.stop()
        #self.cap.release()


class MotorControler:
    from enum import Enum
    # ~ from piwars2024 import set_motor,set_speed,get_encoder,pid
    import piwars2024
    
    class Mode(Enum):
        MANUAL=0
        AUTONOMOUS=1
    class Directions(Enum):
        FORWARD=0
        BACKWARD=1
        STANDING_ROT_LEFT=2
        STANDING_ROT_RIGHT=3
        TURN_LEFT=4
        TURN_RIGHT=5

    def __init__(
            self,
            run=True,
            moveOffset=10,
            turnRadius=.7,          #1=100%
            lowPassFilter=(.25,     #Throughpass
                           .75),    #Oldval
            mode=Mode.MANUAL):
        self.mode=mode
        self.run=run
        self.lpf=lowPassFilter
        self.encoderL=0
        self.encoderR=0
        self.moveOffset=moveOffset
        self.motorInc=0
        self.turnRadius=turnRadius
        self.move=(0,0)
        self.obstasclePos=(0,0)
        #self.startingPoint=self.get_encoder() #Left,Right encoder offset
        self.targetCalculation=0
        # ~ if self.mode==self.Mode.AUTONOMOUS:
            # ~ self.moveThread=t(target=self.continuousMovement,args=(),daemon=True)
            # ~ self.moveThread.start()
        
        limMin = -60.0
        limMax = 60.0
        limMinInt = -1.0
        limMaxInt = 1.0
        T = 0.02
        tau = 0.0
        self.pid = self.piwars2024.pid(0.5, 0.0, 0.00, T, tau, limMin, limMax, limMinInt, limMaxInt)

        self.piwars2024.set_mode(1)

    def setlowPassFilter(self,newLow):
        if newLow<1 and newLow>0:
            self.lowPassFilter=(newLow,1-newLow)
        else:
            print('Info Low pass filter disabled')
            self.lowPassFilter=(1,0)

    def setMotorInc(self,incrementCalculation):
        self.motorInc=incrementCalculation

    def getMotorInc(self):
        return self.motorInc

    def getEncoderValues(self):
        left,right=(0,1)     #index
        return (self.encoderL-self.startingPoint[left],
                self.encoderR-self.startingPoint[right])
    
    def getStartingPoint(self):
        return self.startingPoint
    
    def getLowPassFilter(self):
        return self.lowPassFilter

    # ~ def continuousMovement(self):
        # ~ speed=-25
        # ~ # left,right=(0,1)    #index
        # ~ # low,hi=(0,1)        #index
        # ~ # motorSetting=(0,0)

        # ~ while self.run:
            # ~ measurement=self.move
            # ~ setpoint=0

            # ~ output = self.pid.update(setpoint, measurement)
            # ~ offset += output
            # ~ if offset > 25:
                # ~ offset = 25.0
            # ~ if offset < -25:
                # ~ offset = -25.0

            # ~ lval = int(round(speed + offset))
            # ~ rval = int(round(speed - offset))
            # ~ self.set_speed(lval, rval)
            # self.encoderL,self.encoderR=self.get_encoder()

            # obstacleAvoidance=(
            #     obstacleStart*self.motorInc,
            #     obstacleSize*self.motorInc)

            # motorSetting=(
            #     (motorSetting[left]*self.lpf[low])+(self.move[left]*self.lpf[hi]),
            #     (motorSetting[left]*self.lpf[low])+(self.move[right]*self.lpf[hi]))

            # self.set_motor(
            #     abs(motorSetting[left]),    #Speed for left motor
            #     int(motorSetting[left]>0),  #Direction of left motor
            #     abs(motorSetting[right]),   #Speed of right motor
            #     int(motorSetting[right]>0)) #Direction of right motor

    def moveAutonomousTurn(self, speed):
        self.piwars2024.set_speed(-speed,speed)

    def moveAutonomous(self,delta):#=0,direction=Directions.FORWARD):
        # ~ self.move=delta
        
        measurement=delta
        setpoint=0

        print("measurement={}".format(measurement))

        output = self.pid.update(setpoint, measurement)
#        offset += output
#        if offset > 25:
#            offset = 25.0
#        if offset < -25:
#            offset = -25.0

        lval = int(round(-20 + output))
        rval = int(round(-20 - output))
        self.piwars2024.set_speed(lval, rval)
        
        # low=1*self.turnRadius
        # high=1-low

        # if direction==self.Directions.TURN_RIGHT:
        #     self.move=(speed*high,speed*low)
        # elif direction==self.Directions.TURN_LEFT:
        #     self.move=(speed*low,speed*high)
        # elif direction==self.Directions.STANDING_ROT_RIGHT:
        #     self.move=(speed,-speed)
        # elif direction==self.Directions.STANDING_ROT_LEFT:
        #     self.move=(-speed,speed)
        # elif direction==self.Directions.BACKWARD:
        #     self.move=(-speed,-speed)
        # else: #if direction==self.Directions.FORWARD:
        #     self.move=(speed,speed)

    def moveManual(self):
        pass

    def destroy(self):
        self.run=False
        self.moveThread=0


class Runner:
    from enum import Enum
    from numpy import array_equal,pi
    from time import sleep
    from math import atan,tan
    from piwars2024 import imu_turn
    class Mode(Enum):
        IDLE=0
        SEARCH1=1
        FETCH=2
        SEARCH2=3
        DEPOSIT=4
        VERIFY=5
    class Target(Enum):
        RED=0
        GREEN=1
        BLUE=2
        YELLOW=3
    class Trapezoid(Enum):
        OBTUSE1=0
        ACUTE1=1
        ACUTE2=2
        OBTUSE2=3

    def __init__(self,
                 width=300,
                 sortNum=12,
                 offsetMargin=10,
                 heldObject=80,
                 run=True):
        if DEBUGGING:
            self.debugFrame=[]
            self.debugGreenMask=[]
            self.debugRedMask=[]
        self.cam=Cam()
        self.motorControler=MotorControler()
        self.rotation=(
            0,  #Left num steps
            0)  #Right num steps
        self.numUnsorted=sortNum
        self.numSorted=0
        self.sortGoal=sortNum
        self.offsetMargin=offsetMargin
        self.width=width
        self.heldObject=heldObject
        self.target=None
        self.state=self.Mode.IDLE
        self.run=run
        self.startSignal=False
        self.runThread=t(target=self.runMain,args=(),daemon=True)
        for _ in range(5):
            if self.cam.readSuccess:
                self.cam.setMaskCreation(True)
                break
            else:
                self.sleep(.2)
        if self.cam.readSuccess:
            self.runThread.start()
        else:
            print('Failed to connect camera')
            self.destroy()

    def setStartSignal(self,newSignal):
        self.startSignal=newSignal

    def setState(self,newState):
        self.state=newState
    
    def getState(self):
        return self.state
        
    def calibrate(self):
        #Calculate 1 rotation
        pass

    def evasiveAction(self,movement):
        # self.rotation=((-1*movement),movement)
        self.motorControler.move=(self.motorControler.move+movement)-(self.cam.width/2)

    def avoidOutOfBounds(self,arrayIndex,arrayLen):
        returnValue=arrayLen-1
        if arrayIndex<arrayLen:
            returnValue=arrayIndex
        else:
            if DEBUGGING:
                print('Index out of bounds:',arrayIndex,arrayLen)
            else:
                pass
        if arrayIndex<0:
            if DEBUGGING:
                print('Index amller than 0:',arrayIndex)
            returnValue=0
        return returnValue
   
    def trapezoidAlgoritm(self,trapezoid,l1,l2,roiH,roiL,yVal):
        yAlgR=roiL-l2
        if (trapezoid==self.Trapezoid.OBTUSE2):
            yAlgL=l1
            xAlgL=roiH-yVal
            xAlgR=yVal
        elif (trapezoid==self.Trapezoid.ACUTE1):
            yAlgL=l1
            xAlgL=roiH-yVal
            xAlgR=roiH-yVal
        elif (trapezoid==self.Trapezoid.ACUTE2):
            yAlgL=roiL-l1
            xAlgL=roiH-yVal
            xAlgR=roiH-yVal
        elif (trapezoid==self.Trapezoid.OBTUSE1):
            yAlgL=l1
            xAlgL=yVal
            xAlgR=roiH-yVal

        yAlgL=self.avoidOutOfBounds(int(yAlgL),roiL)
        yAlgR=self.avoidOutOfBounds(int(yAlgR),roiL)
        xAlgL=self.avoidOutOfBounds(int(xAlgL),roiH)
        xAlgR=self.avoidOutOfBounds(int(xAlgR),roiH)

        return (xAlgL,yAlgL,xAlgR,yAlgR)

    def zeroIn(self,obj):
        # delta=goal-obj
        mc=self.motorControler
        measurement=obj
        mc.moveAutonomous(measurement)
        # if abs(delta)<self.offsetMargin:    #Move forward
        #     if DEBUGGING:
        #         print('go')
        #     mc.moveAutonomous(50)
        # else:
        #     if delta<0:
        #         if DEBUGGING:
        #             print('right '+str(abs(delta)))
        #         mc.moveAutonomous(delta,mc.Directions.TURN_RIGHT)
        #     else:
        #         if DEBUGGING:
        #             print('left '+str(delta))
        #         mc.moveAutonomous(delta,mc.Directions.TURN_LEFT)

    #Main run sequence
    def runMain(self):

        while self.run:
            if self.state==self.Mode.IDLE:
                self.idle()
            if self.state==self.Mode.SEARCH1:
                self.search1()
            if self.state==self.Mode.FETCH:
                self.fetch()
            if self.state==self.Mode.SEARCH2:
                self.search2()
            if self.state==self.Mode.DEPOSIT:
                self.deposit()
            if self.state==self.Mode.VERIFY:
                self.verify()

    #Do nothing, wait for start signal
    def idle(self):
        print('IDLE')
        goal=True

        if DEBUGGING:
            print('Target:\t',str(self.target))
            print('camRead:\t',str(self.cam.readSuccess))

        while self.state==self.Mode.IDLE:
            result=self.startSignal
            if result==goal:
                break
        self.setState(self.Mode.SEARCH1)

    #Find objective. If none in view, rotate
    def search1(self,speed=50):
        print('SEARCH1')
        
        centerPoint=0   #index
        offset=1        #index
        area=2          #index
        goal=True
        result=False
        speed=speed

        while self.state==self.Mode.SEARCH1:
            
            time.sleep(0.02)
            rC=self.cam.redCenter
            gC=self.cam.greenCenter

            #Select target
            #If several targets in view
            if ((rC[centerPoint])[0]<self.cam.width and   #Check if x coordinate in view
                (gC[centerPoint])[0]<self.cam.width):     #Check if x coordinate in view
                #Compare area size (proximity)
                if rC[area]>gC[area]:
                    self.target=self.Target.RED
                    if DEBUGGING: 
                        frame=self.cam.getFrame()
                        cv.circle(frame,rC[0],10,(0,0,255),-1)
                        self.debugFrame=frame
                elif rC[area]<gC[area]:                 
                    self.target=self.Target.GREEN
                    if DEBUGGING: 
                        frame=self.cam.getFrame()
                        cv.circle(frame,gC[0],10,(0,255,0),-1)
                        self.debugFrame=frame
                #If equal size, compare offset
                else:
                    if rC[offset]<gC[offset]:
                        self.target=self.Target.RED
                        if DEBUGGING: 
                            frame=self.cam.getFrame()
                            cv.circle(frame,rC[0],10,(0,0,255),-1)
                            self.debugFrame=frame
                    elif rC[offset]>gC[offset]:
                        self.target=self.Target.GREEN
                        if DEBUGGING: 
                            frame=self.cam.getFrame()
                            cv.circle(frame,gC[0],10,(0,255,0),-1)
                            self.debugFrame=frame
                    #If bouth objects are equal in size and offset, select red
                    else:
                        self.target=self.Target.RED
                        if DEBUGGING: 
                            frame=self.cam.getFrame()
                            cv.circle(frame,rC[0],10,(0,0,255),-1)
                            self.debugFrame=frame

            #Only red object in view
            elif ((rC[centerPoint])[0]<self.cam.width and #Check if x coordinate in view
                (gC[centerPoint])[0]>self.cam.width):     #Check if x coordinate in view
                self.target=self.Target.RED
                if DEBUGGING: 
                    frame=self.cam.getFrame()
                    cv.circle(frame,rC[0],10,(0,0,255),-1)
                    self.debugFrame=frame
            #Only green object in view
            elif ((rC[centerPoint])[0]>self.cam.width and #Check if x coordinate in view
                (gC[centerPoint])[0]<self.cam.width):     #Check if x coordinate in view
                self.target=self.Target.GREEN
                if DEBUGGING: 
                    frame=self.cam.getFrame()
                    cv.circle(frame,gC[0],10,(0,255,0),-1)
                    self.debugFrame=frame

            #If no targets in view
            #Search for target
            else:
                if DEBUGGING:
                    print('No target: Turn RIGHT')

                self.motorControler.moveAutonomousTurn(20)
                # self.motorControler.moveAutonomous(speed,
                #     self.motorControler.Directions.STANDING_ROT_RIGHT)
            
            result=self.target!=None
            if result==goal:
                break
        self.setState(self.Mode.FETCH)

    #Center on and go to object 
    def fetch(self):
        print('FETCH')
        print('target='+str(self.target))

        x=0     #Index
        y=1     #Index
        goal=True
        height=self.cam.height
        roiSize=(self.width,self.heldObject)

        i = 0
        while self.state==self.Mode.FETCH:
            print("#{}".format(i))
            i = i + 1
            time.sleep(0.02)
            
            if self.target==self.Target.RED:
                centerPoint,_,_,targetSize,_=self.cam.getRedCenter()
            elif self.target==self.Target.GREEN:
                centerPoint,_,_,targetSize,_=self.cam.getGreenCenter()
            else:
                if DEBUGGING:
                    print('ERROR! fetch state: unknown target')
                    self.setStartSignal(False)
                    self.setState(self.Mode.IDLE)
                else:
                    self.target=None
                    self.setState(self.Mode.SEARCH1)
                break

            #If target is lost, return to search1 state
            if (centerPoint[0]>self.cam.width or
                targetSize[0]<=0 or
                targetSize[1]<=0):
                self.state=self.Mode.SEARCH1
                self.target=None
            
            self.zeroIn(centerPoint[x]-(self.cam.width/2))

            print(centerPoint)

            #Result ok if target inside of roi
            targetRightSide=centerPoint[x]+int(targetSize[x]*.75)
            clawRightSide=self.cam.globalCenter[x]+int(roiSize[x]*.75)
            targetLeftSide=centerPoint[x]-int(targetSize[x]*.75)
            clawLeftSide=self.cam.globalCenter[x]-int(roiSize[x]*.75)
            targetDepth=centerPoint[y]-int(targetSize[y]*.5)
            
            result=targetDepth>height-roiSize[y]
            # ~ result=(targetRightSide<clawRightSide and
                    # ~ targetLeftSide>clawLeftSide and
                    # ~ targetDepth>height-roiSize[y])
            
            if result==goal:
                break

        #Check if mode was changed (wich indicates lost target)
        if self.state==self.Mode.FETCH:
            self.setState(self.Mode.SEARCH2)
            if self.target==self.Target.RED:
                self.target=self.Target.YELLOW
            elif self.target==self.Target.GREEN:
                self.target=self.Target.BLUE

    #Use motorControler.get_encoder() to calculate where to deposit object
    #find deposit color based on object color
    def search2(self):
        print('SEARCH2')

        x=0             #Index
        leftMotor=0     #Index
        rightMotor=1    #Index
        goal=True
        acceptableOffset=self.offsetMargin
        mc=self.motorControler

        while self.state==self.Mode.SEARCH2:
            if self.target==self.Target.BLUE:
                centerPoint,_,_,targetSize,_=self.cam.getBlueCenter()
            elif self.target==self.Target.YELLOW:
                centerPoint,_,_,targetSize,_=self.cam.getYellowCenter()
            else:
                if DEBUGGING:
                    print('ERROR! fetch state: unknown target')
                    self.setStartSignal(False)
                    self.setState(self.Mode.IDLE)
                else:
                    self.target=None
                    self.setState(self.Mode.SEARCH1)
                break

            #Check if deposit is in view
            if targetSize[x]<=0 or centerPoint[x]>self.cam.width:
                move=mc.getEncoderValues()
                if move[leftMotor]>move[rightMotor]:
                    if DEBUGGING:
                        print('Deposit not in view, move right')
                    # mc.moveAutonomous(self.cam.globalCenter[x],mc.Directions.TURN_RIGHT)
                    mc.moveAutonomous(self.cam.width-(self.cam.width/2))
                else:
                    if DEBUGGING:
                        print('Deposit not in view, move left')
                    # mc.moveAutonomous(self.cam.globalCenter[x],mc.Directions.TURN_LEFT)    
                    mc.moveAutonomous(0-(self.cam.width/2))    
            else:
                self.zeroIn(centerPoint[x]-(self.cam.width/2))

            result=(centerPoint[x]>self.cam.globalCenter[x]-acceptableOffset and
                    centerPoint[x]<self.cam.globalCenter[x]+acceptableOffset)
            if result==goal:
                break
        self.setState(self.Mode.DEPOSIT)

    #Move to deposit, back off
    def deposit(self):
        self.imu_turn(0)
        print('DEPOSIT')

        x=0                 #index
        y=1                 #index
        rH,b1,b2=(0,0,0)    #trapezoid algorithm: length, height, bases
        angle1,angle2=(0,0) #trapezoid algorithm: angles
        xAlgL,yAlgL=(1,1)   #trapezoid algorithm left side
        xAlgR,yAlgR=(1,1)   #trapezoid algorithm right side
        trapezoid=-1        #For trapezoid typ identification
        goal=True
        roi=[]

        while self.state==self.Mode.DEPOSIT:            
            if self.target==self.Target.BLUE:
                centerPoint,targetOffset,targetArea,targetSize,startPoint=self.cam.getBlueCenter()
            if self.target==self.Target.YELLOW:
                centerPoint,targetOffset,targetArea,targetSize,startPoint=self.cam.getYellowCenter()
            else:
                print('ERROR! Deposit state: unknown target')
                self.setStartSignal(False)
                self.setState(self.Mode.IDLE)
                break
           
            #Enclose movement route in four points
            #  ptD ____ ptC
            #      \   \
            #       \    \
            #        \     \
            #     ptA \______\ ptB
            ptA=(int(self.cam.globalCenter[x]-(self.width*.5)), #A:X
                 int(self.cam.height-self.heldObject))          #A:Y
            ptB=(int(self.cam.globalCenter[x]+(self.width*.5)), #B:X
                 int(self.cam.height-self.heldObject))          #B:Y
            ptC=(startPoint[x]+targetSize[x],                   #C:X
                 startPoint[y]+targetSize[y])                   #C:Y
            ptD=(startPoint[x],                                 #D:X
                 startPoint[y]+targetSize[y])                   #D:Y

            #Depending on the shape of the route (usually trapezoid, acute or obtuse)
            #A and D or B and C may switch places.
            #       OBTUSE1                 ACUTE                  OBTUSE2
            #  ptD ____ ptC              ptD ____ ptC              ptD ____ ptC
            #      \   \                    /    \                    /   /
            #       \    \                 /      \                 /    /
            #        \     \              /        \              /     / 
            #     ptA \______\ ptB   ptA /__________\ ptB   ptA /______/ ptB
            # D to the left of A      D to the right of A      D to the left of A
            # C to the left of B      C to the left of B       C to the right of B
            if ptA[y]-ptD[y]>0:
                frame=self.cam.getFrame()
                rH=ptA[y]-ptD[y]                    #Hight of ROI

                if ptA[x]<ptD[x]:
                    b1=abs(ptA[x]-ptD[x])           #First triangle base

                    if ptB[x]<ptC[x]:   #OBTUSE2
                        if ptC[x]-ptA[x]>0:
                            trapezoid=self.Trapezoid.OBTUSE2
                            roi=frame[ptD[y]:ptA[y],ptA[x]:ptC[x]]

                            b2=(abs(ptB[x]-ptC[x])) #Second triangle base
                        else:
                            print("INFO! Deposit state: goal lost")
                    else:               #ACUTE
                        if ptC[x]-ptD[x]>0:
                            trapezoid=self.Trapezoid.ACUTE1
                            roi=frame[ptD[y]:ptA[y],ptA[x]:ptB[x]]

                            b2=(abs(ptC[x]-ptB[x])) #Second triangle base
                        else:
                            print("INFO! Deposit state: goal lost")
                else:
                    b1=abs(ptD[x]-ptA[x])           #First triangle base

                    if ptB[x]<ptC[x]:   #ACUTE upside down :)
                        if ptD[x]-ptC[x]!=0:
                            trapezoid=self.Trapezoid.ACUTE2
                            roi=frame[ptD[y]:ptA[y],ptD[x]:ptC[x]]

                            b2=(abs(ptB[x]-ptC[x])) #Second triangle base
                        else:
                            print("INFO! Deposit state: goal lost")
                    else:               #OBTUSE1
                        if ptB[x]-ptD[x]>0:
                            trapezoid=self.Trapezoid.OBTUSE1
                            roi=frame[ptD[y]:ptA[y],ptD[x]:ptB[x]]

                            b2=(abs(ptC[x]-ptB[x])) #Second triangle base
                        else:
                            print("INFO! Deposit state: goal lost")

                if self.array_equal(roi,[]):
                    print('INFO! Deposit state: nothing in ROI')

                angle1,angle2=(
                    (self.atan(b1/rH)*(180/self.pi)),   #angle A in triangle 1
                    (self.atan(b2/rH)*(180/self.pi)))   #angle A in triangle 2

                greenMask,_,_,redMask=self.cam.createColorMasks(roi)
                _,_,_,greenSize,greenStarPoint=self.cam.findCenter(greenMask)
                _,_,_,redSize,redStarPoint=self.cam.findCenter(redMask)

                if greenSize[x]>0:
                    l1=int((greenStarPoint[y]+greenSize[y])*self.tan((self.pi/180)*angle1))
                    l2=int((greenStarPoint[y]+greenSize[y])*self.tan((self.pi/180)*angle2))
                    
                    xAlgL,yAlgL,xAlgR,yAlgR=self.trapezoidAlgoritm(
                        trapezoid,l1,l2,rH,len(roi[0]),greenStarPoint[y]+greenSize[y])
                    
                    if greenStarPoint[x]+greenSize[x]>xAlgL:
                        self.evasiveAction(xAlgL-(greenStarPoint[x]+greenSize[x])) #Will have a negative value
                        if DEBUGGING:
                            greenAlert=(greenStarPoint[x]+greenSize[x],greenStarPoint[y]+greenSize[y])
                            cv.circle(roi,greenAlert,5,(0,0,255),-1)

                    if greenStarPoint[x]<xAlgR:
                        self.evasiveAction(xAlgL-greenStarPoint[x]) #Will have a positive value
                        if DEBUGGING:
                            greenAlert=(greenStarPoint[x],greenStarPoint[y]+greenSize[y])
                            cv.circle(roi,greenAlert,5,(0,0,255),-1)
           
                if redSize[x]>0:
                    l1=int((redStarPoint[y]+redSize[y])*self.tan((self.pi/180)*angle1))
                    l2=int((redStarPoint[y]+redSize[y])*self.tan((self.pi/180)*angle2))
                    
                    xAlgL,yAlgL,xAlgR,yAlgR=self.trapezoidAlgoritm(
                        trapezoid,l1,l2,rH,len(roi[0]),redStarPoint[y]+redSize[y])
                    
                    if greenStarPoint[x]+redSize[x]>xAlgL:
                        self.evasiveAction(xAlgL-(redStarPoint[x]+redSize[x])) #Will have a negative value
                        if DEBUGGING:
                            redAlert=(greenStarPoint[x]+redSize[x],greenStarPoint[y]+redSize[y])
                            cv.circle(roi,redAlert,5,(0,0,255),-1)

                    if redStarPoint[x]<xAlgR:
                        self.evasiveAction(xAlgL-redStarPoint[x]) #Will have a positive value
                        if DEBUGGING:
                            redAlert=(redStarPoint[x],redStarPoint[y]+redSize[y])
                            cv.circle(roi,redAlert,5,(0,0,255),-1)

                if DEBUGGING:
                    for yVal in range(rH):
                        l1=int(yVal*self.tan((self.pi/180)*angle1))
                        l2=int(yVal*self.tan((self.pi/180)*angle2))

                        xAlgL,yAlgL,xAlgR,yAlgR=self.trapezoidAlgoritm(trapezoid,l1,l2,rH,len(roi[0]),yVal)
                        
                        roi[xAlgL][yAlgL]=(255,255,0)
                        roi[xAlgR][yAlgR]=(255,0,255)
                    
                    self.debugFrame=roi
                    self.debugGreenMask=greenMask
                    self.debugRedMask=redMask

                self.motorControler.moveAutonomous()

                result=('''object in target''')
                if result==goal:
                    self.numUnsorted=self.numUnsorted-1
                    self.numSorted=self.numSorted+1
                    self.target=None
                    break

            else:
                print('INFO! Deposit state: no target')

        self.setState(self.Mode.VERIFY)

    #Check if objectiva has been achieved
    def verify(self):
        print('VERIFY')
        goal=self.sortGoal
        while self.state==self.Mode.VERIFY:
            result=self.numSorted
            if result==goal:
                self.startSignal=False
                self.setState(self.Mode.IDLE)
            else:
                self.setState(self.Mode.SEARCH1)

    def destroy(self):
        self.run=False
        self.runThread=0
        self.cam.destroy()
        self.motorControler.destroy()        
