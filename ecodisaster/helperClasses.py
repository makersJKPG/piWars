from threading import Thread as t
from cv2 import (
    cvtColor,putText,morphologyEx,findContours,contourArea,
    boundingRect,inRange,bitwise_or,#VideoWriter_fourcc,VideoCapture,
    #CAP_DSHOW,CAP_PROP_FRAME_WIDTH,CAP_PROP_FRAME_HEIGHT,
    #CAP_PROP_FPS,CAP_PROP_FOURCC,
    FONT_HERSHEY_COMPLEX,COLOR_RGB2BGR,MORPH_OPEN,
    MORPH_CLOSE,CHAIN_APPROX_SIMPLE,RETR_EXTERNAL,COLOR_BGR2HSV)
from numpy import array,array_equal,uint8,ones,zeros
from math import sqrt


DEBUGGING=True
if DEBUGGING:
    import cv2 as cv

class Cam:
    from picamera2 import Picamera2

    def __init__(
            self,
            #camNum=0,
            capFormat='XRGB8888',
            capWidth=int(640*.25),
            capHeight=int(480*.25),
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
        self.cap.configure(self.cap.create_preview_configuration(main={"format":capFormat,"size": (capWidth,capHeight)}))
        self.cap.start()
        self.longestPossibleOffset=sqrt(pow(capWidth*.5,2)+pow(capHeight*.5,2))
        self.globalCenter=(int(capWidth*.5),int(capHeight*.5))
        #self.cap.set(CAP_PROP_FRAME_WIDTH,capWidth)
        #self.cap.set(CAP_PROP_FRAME_HEIGHT,capHeight)
        #self.cap.set(CAP_PROP_FPS,fps)
        #self.cap.set(CAP_PROP_FOURCC,codec)
        self.frame=[]
        self.greenMask=array([])
        self.greenCenter=(0,0)
        self.redMask=array([])
        self.redCenter=(0,0)
        self.blueMask=array([])
        self.blueCenter=(0,0)
        self.yellowMask=array([])
        self.yellowCenter=(0,0)
        self.kernel=ones((5,5),uint8)
        self.readSuccess=False
        self.createMasks=False
        self.emptyFrame=zeros((capWidth,capHeight),uint8)
        putText(
            self.emptyFrame,'Error reading frame',(25,25), 
            FONT_HERSHEY_COMPLEX,1,(255,255,255), 1)
        self.run=True
        self.capThread=t(target=self.captureFrame,args=(),daemon=True)
        # self.cvtThread=t(target=self.createColorMasks,args=(),daemon=True)
        # self.cvtThread.start()
        self.capThread.start()
        #print('Width:\t'+str(self.cap.get(CAP_PROP_FRAME_WIDTH)))
        #print('Height:\t'+str(self.cap.get(CAP_PROP_FRAME_HEIGHT)))

    def setMaskCreation(self,createMask):
        self.createMasks=createMask

    def setSize(self,newWidth,newHeight):
        self.cap.set(CAP_PROP_FRAME_WIDTH,newWidth)
        self.cap.set(CAP_PROP_FRAME_HEIGHT,newHeight)

    def getFrame(self):
        if self.readSuccess:
            return self.frame
        else:
            return cvtColor(self.emptyFrame,COLOR_RGB2BGR)

    def getFrameSize(self):
        return (self.cap.get(CAP_PROP_FRAME_WIDTH),self.cap.get(CAP_PROP_FRAME_HEIGHT))

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


class Runner:
    from enum import Enum
    from numpy import array_equal,pi
    from time import sleep
    from math import atan,tan
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

    def __init__(
            self,
            width=300/640,
            sortNum=12,
            offsetMargin=10/640,
            heldObject=70/640):
            
        if DEBUGGING:
            self.debugFrame=[]
            self.debugGreenMask=[]
            self.debugRedMask=[]
        self.cam=Cam()
        self.rotation=(
            0,  #Left num steps
            0)  #Right num steps
        self.numUnsorted=sortNum
        self.numSorted=0
        self.sortGoal=sortNum
        self.offsetMarginX=int(offsetMargin*self.cam.width)
        self.offsetMarginY=int(offsetMargin*self.cam.height)
        self.width=int(width*self.cam.width)
        self.heldObject=int(heldObject*self.cam.width)
        self.target=None
        self.state=self.Mode.IDLE
        self.startSignal=False
        self.runThread=t(target=self.run,args=(),daemon=True)
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
        self.rotation=((-1*movement),movement)

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

    def zeroIn(self,obj,goal):
        if DEBUGGING:
            pass
        else:
            delta=goal-obj
            if abs(delta)<self.offsetMarginX:
                print('go')
            else:
                if delta<0:
                    print('right '+str(abs(delta)))
                else:
                    print('left '+str(delta))

    #Main run sequence
    def run(self):

        while True:
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
    def search1(self):
        print('SEARCH1')
        
        centerPoint=0   #index
        offset=1        #index
        area=2          #index
        goal=True
        result=False

        while self.state==self.Mode.SEARCH1:
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
                print('No target: Turn RIGHT')
                self.rotation=(self.rotation[0]+10,self.rotation[1])
            
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

        while self.state==self.Mode.FETCH:
            if self.target==self.Target.RED:
                centerPoint,_,_,targetSize,_=self.cam.getRedCenter()
            elif self.target==self.Target.GREEN:
                centerPoint,_,_,targetSize,_=self.cam.getGreenCenter()
            else:
                print('ERROR! fetch state: unknown target')
                self.setStartSignal(False)
                self.setState(self.Mode.IDLE)
                break

            #If target is lost, return to search1 state
            if (centerPoint[0]>self.cam.width or
                targetSize[0]<=0 or
                targetSize[1]<=0):
                self.state=self.Mode.SEARCH1
                self.target=None
            
            self.zeroIn(centerPoint[x],self.cam.globalCenter[x])

            #Result ok if target inside of roi
            result=(centerPoint[x]+int(targetSize[x]*.5)<self.cam.globalCenter[x]+int(roiSize[x]*.5) and
                    centerPoint[x]-int(targetSize[x]*.5)>self.cam.globalCenter[x]-int(roiSize[x]*.5) and
                    centerPoint[y]+int(targetSize[y]*.5)<height and
                    centerPoint[y]-int(targetSize[y]*.5)>height-roiSize[y])
            
            if result==goal:
                break

        #Check if mode was changed (wich indicates lost target)
        if self.state==self.Mode.FETCH:
            self.setState(self.Mode.SEARCH2)
            if self.target==self.Target.RED:
                self.target=self.Target.YELLOW
            elif self.target==self.Target.GREEN:
                self.target=self.Target.BLUE

    #Use self.rotation to calculate where to deposit object
    #find deposit color based on object color
    def search2(self):
        print('SEARCH2')

        x=0             #Index
        leftMotor=0     #Index
        rightMotor=1    #Index
        goal=True
        acceptableOffset=self.offsetMarginX
        move=(self.rotation[0]*-1,self.rotation[1]*-1)

        while self.state==self.Mode.SEARCH2:
            if self.target==self.Target.BLUE:
                centerPoint,_,_,targetSize,_=self.cam.getBlueCenter()
            elif self.target==self.Target.YELLOW:
                centerPoint,_,_,targetSize,_=self.cam.getYellowCenter()
            else:
                print('ERROR! search2 state: unknown target')
                self.setStartSignal(False)
                self.setState(self.Mode.IDLE)
                break

            #Check if deposit is in view
            if targetSize[x]<=0 or centerPoint[x]>self.cam.width:
                if move[leftMotor]>move[rightMotor]:
                    print('Deposit not in view, move right')
                else:
                    print('Deposit not in view, move left')
            else:
                self.zeroIn(centerPoint[x],self.cam.globalCenter[x])

            result=(centerPoint[x]>self.cam.globalCenter[x]-acceptableOffset and
                    centerPoint[x]<self.cam.globalCenter[x]+acceptableOffset)
            if result==goal:
                break
        self.setState(self.Mode.DEPOSIT)

    #Move to deposit, back off
    def deposit(self):
        print('DEPOSIT')

        x=0     #index
        y=1     #index
        rH,b1,b2=(0,0,0)   #trapezoid algorithm: length, height, bases
        angle1,angle2=(0,0)     #trapezoid algorithm: angles
        xAlgL,yAlgL=(1,1)       #trapezoid algorithm left side
        xAlgR,yAlgR=(1,1)       #trapezoid algorithm right side
        trapezoid=-1            #For trapezoid typ identification
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
        self.runThread=0
        self.cam.destroy()
        
