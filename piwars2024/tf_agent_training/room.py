
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import abc
import tensorflow as tf
import numpy as np

from ChallengeObjects import Barrel, PiBot
from random import randint
from vpython import box, color, vector, canvas
from tf_agents.environments import py_environment
from tf_agents.environments import tf_environment
from tf_agents.environments import tf_py_environment
from tf_agents.environments import utils
from tf_agents.specs import array_spec
from tf_agents.environments import wrappers
from tf_agents.environments import suite_gym
from tf_agents.trajectories import time_step as ts

arenaColor=vector(.15,.15,.15)
canvas(background=vector(1,1,1))

class ChallengeRoom(py_environment.PyEnvironment):
    def __init__(
        self,
        challengeName,  #Environment spec
        difficulty=6    #Ingeger (1-6)
    ):

        if difficulty<1:
            difficulty=1
        if difficulty>6:
            difficulty=6

        self._action_spec=array_spec.BoundedArraySpec(
            shape=(),dtype=np.int32,minimum=0,maximum=4,name='action')
        self._observation_spec=array_spec.BoundedArraySpec(
            shape=(1,),dtype=np.int32,minimum=0,name='observation')
        self._state=0
        self._episode_ended=False
        self.objectiveCompleted=False

        self.mainHeight=300
        self.thickness=10

        #Specifics for Eco-Disaster challenge
        if challengeName=='Eco-Disaster':
            self.mainWidth=2200
            self.mainLength=2200
            self.cleanZoneWidth=600
            self.cleanZoneWLength=200
            self.contaminatedZoneWidth=600
            self.contaminatedZoneLength=200
            self.startZoneWidth=225
            self.startZoneLength=400
            self.startZoneCenter=vector(0,400-(0.5*self.mainLength),0)
            self.barrelBoxWidth=1600
            self.barrelBoxLength=1600
            
            #floor
            box(
                pos=vector(0,0,-self.thickness*0.5),
                color=arenaColor,
                size=vector(self.mainWidth,self.mainLength,self.thickness,))

            #walls
            self.leftBoundary=vector(-self.mainWidth*0.5,0,self.mainHeight*0.5)
            box(
                pos=self.leftBoundary,
                color=arenaColor,
                size=vector(self.thickness,self.mainLength,self.mainHeight))
            self.frontBoundary=vector(0,-self.mainWidth*0.5,self.mainHeight*0.5)
            box(
                pos=self.frontBoundary,
                color=arenaColor,
                size=vector(self.mainWidth,self.thickness,self.mainHeight))
            self.rightBoundary=vector(self.mainWidth*0.5,0,self.mainHeight*0.5)
            box(
                pos=self.rightBoundary,
                color=arenaColor,
                size=vector(self.thickness,self.mainLength,self.mainHeight))
            self.backBoundary=vector(0,self.mainWidth*0.5,self.mainHeight*0.5)
            box(
                pos=self.backBoundary,
                color=arenaColor,
                size=vector(-self.mainWidth,self.thickness,self.mainHeight))

            #clean zone
            self.cleanZone=box(
                pos=vector(-400,self.mainLength*0.5-self.cleanZoneWLength+self.cleanZoneWLength*0.5,0.1),
                color=color.blue,
                size=vector(self.cleanZoneWidth,self.cleanZoneWLength,self.thickness))
            box(
                pos=vector(-400,self.mainLength*0.5-0.1,self.mainHeight*0.5),
                color=color.blue,
                size=vector(self.cleanZoneWidth,self.thickness,self.mainHeight))

            self.cleanZoneLeftBoundary=self.cleanZone.pos.x-self.cleanZoneWidth/2
            self.cleanZoneRightBoundary=self.cleanZone.pos.x+self.cleanZoneWidth/2
            self.greenBarrelsInCleanZone=[]

            #contamination zone
            self.contaminationZone=box(
                pos=vector(400,self.mainLength*0.5-self.contaminatedZoneLength*0.5,0),
                color=color.yellow,
                size=vector(self.contaminatedZoneWidth,self.contaminatedZoneLength,self.thickness))
            box(
                pos=vector(400,self.mainLength*0.5-0.1,self.mainHeight*0.5),
                color=color.yellow,
                size=vector(self.contaminatedZoneWidth,self.thickness,self.mainHeight))

            self.contaminatedZoneLeftBound=self.contaminationZone.pos.x-self.contaminatedZoneWidth/2
            self.contaminatedZoneRightBound=self.contaminationZone.pos.x+self.contaminatedZoneWidth/2
            self.redBarrelsInContaminatedZone=[]

            #Place challenge objects
            self.barrels=[]
            for num in range(difficulty):
                self.barrels.append(
                    Barrel(color=color.red,startPos=vector(randint(-800,800),randint(-800,800),0)))
                self.barrels.append(
                    Barrel(color=color.green,startPos=vector(randint(-800,800),randint(-800,800),0)))

            #Place PiBot in start box
            self.bot=[PiBot(startPos=self.startZoneCenter)]

    #This will move the 'player'-bot. Checks for collitions with walls and barrels
    def movePiBot(
        self,               #Self
        dir,                #Direction to move in
        speedMultiplier=1.0 #Control for movement speed
        ):
        #Calculate x, y for new position
        x=self.bot[0].body[0].pos.x+(
            self.bot[0].speed*speedMultiplier*(
                ((-self.bot[0].body[0].axis.y))/(self.bot[0].width)*self.bot[0].rotationAngle))
        y=self.bot[0].body[0].pos.y+(
            self.bot[0].speed*speedMultiplier*(
                ((self.bot[0].body[0].axis.x))/(self.bot[0].width)*self.bot[0].rotationAngle))

        #Check new coordinates for wall collition
        testedX,testedY,collition=self.checkCollisionWall(self.bot[0].body[0],x,y,self.bot[0].hitbox)
        if not collition:
            #Check for barrel collition
            self.checkCollisionObjects(testedX,testedY)
        #Move
        self.bot[0].move(dir,testedX,testedY)

    #This will move barrels. Checks for collitions with walls and other barrels
    def moveBarrel(
        self,   #Self
        barrel, #Barrel to move
        x,y     #x,y of next position
        ):
        testedX,testedY,collition=self.checkCollisionWall(barrel[0].body[0],x,y,barrel[0].hitbox)
        if not collition:
            self.checkCollisionObjects(testedX,testedY)
        barrel[0].move(dir,testedX,testedY)
        self.checkIfObjectiveCompleted()

    #Not done... #TODO
    def checkCollisionObjects(self,x,y):
        pass

    #Handle objects colliding with walls (stoping them from leaving the room)
    #Returns (x,y)      x and y for next position. Will individually set them 
    #                   back to current position if collition occurs 
    #        (result)   Bool if collition was detected or not
    def checkCollisionWall(
        self,   #Self
        obj,    #Object to test
        x,y,    #Position to move to
        hitbox  #A multiplier to get object as close as possible to a wall without falling through
        ):
        if x>self.leftBoundary.x+obj.length*hitbox and x<self.rightBoundary.x-obj.length*hitbox:
            result=False
        else:
            result=True
            x=obj.pos.x
        if y>self.frontBoundary.y+obj.length*hitbox and y<self.backBoundary.y-obj.length*hitbox:    
            result=False
        else: 
            result=True
            y=obj.pos.y
        return (x,y,result)

    #Track barrels pushed in and out of goal-zones
    #Returns True if objective is complete, False if not
    def checkIfObjectiveCompleted(self):
        for barrel in self.barrels:
            barrelPos=barrel.body[1].pos
            if barrelPos.y+(barrel.width*0.5)>self.contaminatedZoneLength:
                # I barrel is green, check if it is in clean zone
                if barrel.color==color.green and barrel not in self.greenBarrelsInCleanZone:
                    if barrelPos.x-(barrel.width*0.5)>self.cleanZoneLeftBoundary: 
                        if barrelPos.x+(barrel.width*0.5)<self.cleanZoneRightBoundary:
                            self.greenBarrelsInCleanZone.append(barrel)
                # I barrel is red, check if it is in contaminated zone            
                if barrel.color==color.red and barrel not in self.redBarrelsInContaminatedZone:
                    if barrelPos.x-(barrel.width*0.5)>self.contaminatedZoneLeftBound:  
                        if barrelPos.x+(barrel.width*0.5)<self.contaminatedZoneRightBound:
                            self.redBarrelsInContaminatedZone.append(barrel)
        
        #Remove barrels that is pushed out of clean zone 
        for barrel in self.greenBarrelsInCleanZone:
            barrelPos=barrel.bodu[1].pos
            barrelIndex=self.greenBarrelsInCleanZone.index(barrel)
            if not barrelPos.y+(barrel.width*0.5)>self.contaminatedZoneLength: 
                self.greenBarrelsInCleanZone.pop(barrelIndex)
            if not barrelPos.x-(barrel.width*0.5)>self.cleanZoneLeftBoundary: 
                self.greenBarrelsInCleanZone.pop(barrelIndex)
            if not barrelPos.x+(barrel.width*0.5)<self.cleanZoneRightBoundary:
                self.greenBarrelsInCleanZone.pop(barrelIndex)

        #Remove barrels that is pushed out of contaminated zone 
        for barrel in self.redBarrelsInContaminatedZone:
            barrelPos=barrel.bodu[1].pos
            barrelIndex=self.redBarrelsInContaminatedZone.index(barrel)
            if barrelPos.y+(barrel.width*0.5)>self.contaminatedZoneLength:
                self.redBarrelsInContaminatedZone.pop(barrelIndex)
            if barrelPos.x-(barrel.width*0.5)>self.contaminatedZoneLeftBound:  
                self.redBarrelsInContaminatedZone.pop(barrelIndex)
            if barrelPos.x+(barrel.width*0.5)<self.contaminatedZoneRightBound:
                self.redBarrelsInContaminatedZone.pop(barrelIndex)

        #Check for completion
        if len(self.greenBarrelsInCleanZone)+len(self.redBarrelsInContaminatedZone)==len(self.barrels):
            return True
        else:
            return False

    ## Functions to be used by tf_agent training
    def action_spec(self):
        return self._action_spec

    def observation_spec(self):
        return self._observation_spec
        
    def _reset(self):
        self._state=0
        self._episode_ended=False
        return ts.restart(np.array([self._state],dtype=np.int32))

    def setObjectiveCompleted(self,completion):
        self.objectiveCompleted=completion

    def _step(self,action):
        if self._episode_ended:
            return self.reset()
        if self.objectiveCompleted:
            self._episode_ended=True

        #TODO: make if statements for all actions

        else:
            raise ValueError('Invalid action, '+str(action))

        if self.objectiveCompleted:
            #TODO: callculate and set reward
            reward=1.0
            return st.termination(np.array([self._state],dtype=np.int32),reward)
        else:
            return st.termination(np.array([self._state],dtype=np.int32),reward=0.0,discount=1.0)
            
