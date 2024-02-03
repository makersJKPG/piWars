
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
        challengeName
    ):
        self._action_spec=array_spec.BoundedArraySpec(
            shape=(),dtype=np.int32,minimum=0,maximum=4,name='action')
        self._observation_spec=array_spec.BoundedArraySpec(
            shape=(1,),dtype=np.int32,minimum=0,name='observation')
        self._state=0
        self._episode_ended=False
        self.objectiveCompleted=False

        self.mainHeight=300
        self.thickness=10
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
            self.drumBoxWidth=1600
            self.drumBoxLength=1600
            
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
            box(
                pos=vector(-400,self.mainLength*0.5-self.cleanZoneWLength+self.cleanZoneWLength*0.5,0.1),
                color=color.blue,
                size=vector(self.cleanZoneWidth,self.cleanZoneWLength,self.thickness))
            box(
                pos=vector(-400,self.mainLength*0.5-0.1,self.mainHeight*0.5),
                color=color.blue,
                size=vector(self.cleanZoneWidth,self.thickness,self.mainHeight))

            #contamination zone
            box(
                pos=vector(400,self.mainLength*0.5-self.contaminatedZoneLength*0.5,0),
                color=color.yellow,
                size=vector(self.contaminatedZoneWidth,self.contaminatedZoneLength,self.thickness))
            box(
                pos=vector(400,self.mainLength*0.5-0.1,self.mainHeight*0.5),
                color=color.yellow,
                size=vector(self.contaminatedZoneWidth,self.thickness,self.mainHeight))

            #Place challenge objects
            self.barrels=[
                Barrel(color=color.red,startPos=vector(randint(-800,800),randint(-800,800),0)),
                Barrel(color=color.red,startPos=vector(randint(-800,800),randint(-800,800),0)),
                Barrel(color=color.red,startPos=vector(randint(-800,800),randint(-800,800),0)),
                Barrel(color=color.red,startPos=vector(randint(-800,800),randint(-800,800),0)),
                Barrel(color=color.red,startPos=vector(randint(-800,800),randint(-800,800),0)),
                Barrel(color=color.red,startPos=vector(randint(-800,800),randint(-800,800),0)),
                Barrel(color=color.green,startPos=vector(randint(-800,800),randint(-800,800),0)),
                Barrel(color=color.green,startPos=vector(randint(-800,800),randint(-800,800),0)),
                Barrel(color=color.green,startPos=vector(randint(-800,800),randint(-800,800),0)),
                Barrel(color=color.green,startPos=vector(randint(-800,800),randint(-800,800),0)),
                Barrel(color=color.green,startPos=vector(randint(-800,800),randint(-800,800),0)),
                Barrel(color=color.green,startPos=vector(randint(-800,800),randint(-800,800),0))]
            #Place PiBot in start box
            self.bot=[PiBot(startPos=self.startZoneCenter)]

    def movePiBot(self,dir,speedMultiplier=1.0):
        x=self.bot[0].body[0].pos.x+(
            self.bot[0].speed*speedMultiplier*(((-self.bot[0].body[0].axis.y))/(self.bot[0].width)*self.bot[0].rotationAngle))
        y=self.bot[0].body[0].pos.y+(
            self.bot[0].speed*speedMultiplier*(((self.bot[0].body[0].axis.x))/(self.bot[0].width)*self.bot[0].rotationAngle))

        testedX,testedY=self.checkCollisionWall(self.bot[0].body[0],x,y)
        self.bot[0].move(dir,testedX,testedY)

    def checkCollisionWall(self,obj,x,y):
        if x>self.leftBoundary.x+obj.length*0.9 and x<self.rightBoundary.x-obj.length*0.9:
            pass
        else:
            x=obj.pos.x
        if y>self.frontBoundary.y+obj.length*0.9 and y<self.backBoundary.y-obj.length*0.9:    
            pass
        else: 
            y=obj.pos.y
        return (x,y)

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