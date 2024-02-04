from vpython import cylinder,box,ring,vector
import numpy as np

piBotSize=vector(120,200,80)
barrelSize=(
    vector(80,50,50),   # Main cylinder
    3,                  # Ring thickness
    25)                 # Ring radius

class ChallengeObject:
    def __init__(
        self,
        startPos=vector(0,0,0),
        color=vector(0,0,0),
        size=vector(0,0,0)
        ):
            self.position=startPos
            self.rotationAngle=np.pi/16
            self.width=size.x
            self.length=size.y
            self.height=size.z
            self.color=color

    def move(self,direction,newX,newY):
        for part in self.body:
            part.pos.x=newX
            part.pos.y=newY
            if direction=='left':
                part.rotate(axis=vector(0,0,1),angle=self.rotationAngle,origin=part.pos)
            elif direction=='right':
                part.rotate(axis=vector(0,0,1),angle=-self.rotationAngle,origin=part.pos)
       
    def tilt(self):
        self.axis=newAxisMove

class PiBot(ChallengeObject):
    def __init__(
        self,
        color=vector(1,0,1),
        startPos=vector(0,0,0)):
        super().__init__(
            startPos=startPos,
            color=color,
            size=piBotSize
        )

        heightOffset=vector(0,0,0.5*piBotSize.z)
       
        self.hitbox=0.9
        self.body=[box(
            pos=startPos+heightOffset,
            size=piBotSize,
            color=color)]

        self.speed=150


class Barrel(ChallengeObject):
    def __init__(
        self,
        color=vector(0,0,0),
        startPos=vector(0,0,0)):

        super().__init__(
            startPos=startPos,
            color=color,
            size=barrelSize[0]
        )

        ringOffsetTop=vector(0,0,77)
        ringOffsetMid=vector(0,0,41.5)
        ringOffsetBot=vector(0,0,3)
        
        self.hitbox=0.28
        self.body=[
            cylinder(
                size=barrelSize[0],
                color=color,
                axis=vector(0,0,1),
                pos=startPos),
            ring(
                radius=barrelSize[2],
                thickness=barrelSize[1],
                axis=vector(0,0,1),
                color=color,
                pos=startPos+ringOffsetTop),
            ring(
                radius=barrelSize[2],
                thickness=barrelSize[1],
                axis=vector(0,0,1),
                color=color,
                pos=startPos+ringOffsetMid),
            ring(
                radius=barrelSize[2],
                thickness=barrelSize[1],
                axis=vector(0,0,1),
                color=color,
                pos=startPos+ringOffsetBot)]
