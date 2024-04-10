import cv2 as cv
import time
from helperClasses import Cam,Runner
import numpy as np
import math

testRunner=Runner()

# cam=Cam()
# while not cam.readSuccess:
#     time.sleep(.05)
# print('Camera connected')

# fW,fH=cam.getFrameSize()

# cam.setMaskCreation(True)
# time.sleep(.05)
time.sleep(1)
#testRunner.setStartSignal(True)

# def zeroIn(obj,goal):
#     delta=goal-obj
#     if abs(delta)<10:
#         print('go')
#     else:
#         if delta<0:
#             print('right '+str(abs(delta)))
#         else:
#             print('left '+str(delta))
gCenter=testRunner.cam.globalCenter
while testRunner.cam.readSuccess:
    frame=testRunner.cam.getFrame()
    
    find = testRunner.cam.runInference()

    if not find=={}:
        print('inference')
        x=find['x']
        y=find['y']
        w=find['w']
        h=find['h']
        cl=find['class']
        if cl==1:
            c=(0,255,0)
        if cl==2:
            c=(0,0,255)        
        cv.rectangle(frame,(x,y),(x+w,y+h),c,3)
    else:
        print('no inference')
    
    #centerPoint,targetOffset,targetArea,targetSize,startPoint=testRunner.cam.getBlueCenter()
    # print('startP:',startPoint)

    # x,y=(0,1)
    # ptA=(int(testRunner.cam.globalCenter[x]-(testRunner.width*.5)), #A:X
    #         testRunner.cam.height-testRunner.heldObject)                               #A:Y
    # ptB=(int(testRunner.cam.globalCenter[x]+(testRunner.width*.5)), #B:X
    #         testRunner.cam.height-testRunner.heldObject)                               #B:Y
    # ptC=(startPoint[x]+targetSize[x],                                 #C:X
    #         startPoint[y]+int(targetSize[y]))                   #C:Y
    # ptD=(startPoint[x],                   #D:X
    #         startPoint[y]+int(targetSize[y]))                   #D:Y

    # print('ptA:',str(ptA[x]))
    # print('ptB:',str(ptB[x]))
    # print('ptC:',str(ptC[x]))
    # print('ptD:',str(ptD[x]))

    # print('comp:',str(ptA[x]),'<',str(ptD[x]),'=',ptA[x]<ptD[x])
    # print('comp:',str(ptB[x]),'<',str(ptC[x]),'=',ptB[x]<ptC[x])

    # cv.circle(frame,ptA,5,(255,255,0),-1)
    # cv.circle(frame,ptB,5,(0,255,255),-1)
    # cv.circle(frame,ptC,5,(255,0,255),-1)
    # cv.circle(frame,ptD,5,(255,255,255),-1)

    # trap='N/A'
    # if ptA[x]<ptD[x]:
    #     if ptB[x]<ptC[x]:   #Obtuse2
    #         trap='obtuse2'
    #         if ptD[y]-ptA[y]!=0 and (ptA[x])-ptC[x]!=0:
    #             roi=frame[ptD[y]:ptA[y],ptA[x]:ptC[x]]
    #     else:               #Acute
    #         trap='acute'
    #         if ptD[y]-ptA[y]!=0 and (ptA[x])-ptB[x]!=0:
    #             roi=frame[ptD[y]:ptA[y],ptA[x]:ptB[x]]
    # else:
    #     if ptB[x]<ptC[x]:   #Acute upside down :)
    #         trap='acute UD'
    #         if ptD[y]-ptA[y]!=0 and ptD[x]-ptC[x]!=0:
    #             roi=frame[ptD[y]:ptA[y],ptD[x]:ptC[x]]
    #     else:               #Obtuse1
    #         trap='obtuse1'
    #         if ptD[y]-ptA[y]!=0 and (ptD[x])-ptB[x]!=0:
    #             roi=frame[ptD[y]:ptA[y],ptD[x]:ptB[x]]
    # cv.putText(roi,trap,(15,15),1,1,(222,222,222),1,)
    # roi=testRunner.debugFrame

    """     ycenterPoint,ytargetOffset,ytargetArea,ytargetSize,ystartPoint=testRunner.cam.getYellowCenter()
    gcenterPoint,gtargetOffset,gtargetArea,gtargetSize,gstartPoint=testRunner.cam.getGreenCenter()
    rcenterPoint,rtargetOffset,rtargetArea,rtargetSize,rstartPoint=testRunner.cam.getRedCenter()
    pt2=(startPoint[0]+targetSize[0],startPoint[1]+targetSize[1])
    pt1=(startPoint[0],startPoint[1]+targetSize[1])
    cv.line(frame,(gCenter[0]-int(testRunner.width*.5),testRunner.cam.height),pt1,(255,0,0),1)
    cv.line(frame,(gCenter[0]+int(testRunner.width*.5),testRunner.cam.height),pt2,(255,0,0),1)

    ypt2=(ystartPoint[0]+ytargetSize[0],ystartPoint[1]+ytargetSize[1])
    ypt1=(ystartPoint[0],ystartPoint[1]+ytargetSize[1])
    cv.line(frame,(gCenter[0]-int(testRunner.width*.5),testRunner.cam.height),ypt1,(0,255,255),1)
    cv.line(frame,(gCenter[0]+int(testRunner.width*.5),testRunner.cam.height),ypt2,(0,255,255),1)

    gpt2=(gstartPoint[0]+gtargetSize[0],gstartPoint[1]+gtargetSize[1])
    gpt1=(gstartPoint[0],gstartPoint[1]+gtargetSize[1])
    cv.line(frame,(gCenter[0]-int(testRunner.width*.5),testRunner.cam.height),gpt1,(0,255,0),1)
    cv.line(frame,(gCenter[0]+int(testRunner.width*.5),testRunner.cam.height),gpt2,(0,255,0),1)

    rpt2=(rstartPoint[0]+rtargetSize[0],rstartPoint[1]+rtargetSize[1])
    rpt1=(rstartPoint[0],rstartPoint[1]+rtargetSize[1])
    cv.line(frame,(gCenter[0]-int(testRunner.width*.5),testRunner.cam.height),rpt1,(0,0,255),1)
    cv.line(frame,(gCenter[0]+int(testRunner.width*.5),testRunner.cam.height),rpt2,(0,0,255),1)

    cv.rectangle(
        frame,
        (int(testRunner.cam.globalCenter[0]-testRunner.width*.5),testRunner.cam.height-testRunner.heldObject),
        (int(testRunner.cam.globalCenter[0]+testRunner.width*.5),testRunner.cam.height-1),
        (255,255,255),
        3)

    if not np.array_equal(testRunner.debugFrame,[]):
        cv.imshow('target',testRunner.debugFrame)
    if not np.array_equal(testRunner.debugGreenMask,[]):
        cv.imshow('green',testRunner.debugGreenMask)
    if not np.array_equal(testRunner.debugRedMask,[]):
        cv.imshow('red',testRunner.debugRedMask)
 """
    cv.imshow('Normal frame',frame)

    if cv.waitKey(1)==ord('q'):
        print('Quit')
        break

# cam.destroy()
testRunner.destroy()
