import piwars2024
import struct
import math

pack_format = "ihBB"
infile_path = "/dev/input/js0"
event_size = struct.calcsize(pack_format)

file = open(infile_path, "rb", buffering=0)

DPAD_UPDOWN = 7
DPAD_LEFTRIGHT = 6
JOY1_UPDOWN = 1
JOY1_LEFTRIGHT = 0
JOY2_UPDOWN = 4
JOY2_LEFTRIGHT = 3
LEFT_TRIGGER = 2
RIGHT_TRIGGER = 5

def drive(speed, turning):

    lval = abs(speed)
    rval = abs(speed)
    if speed > 0:
        ldir = 1
        rdir = 0
    else:
        ldir = 0
        rdir = 1

    angle = turning*math.pi/2/200
    scale = math.cos(angle)
    if angle > 0:
        rval = int(lval * scale)
    else:
        lval = int(rval * scale)

    piwars2024.set_motor(lval, ldir, rval, rdir)

turning = 0
speed = 0

piwars2024.set_mode(2)

while True:
        
    event = file.read(event_size)
    styring = struct.unpack(pack_format, event)
    
    milli, value, action, button = styring
    #print("milli={} value={} action={} button={}".format(milli, value, action, button))
    
    if button == DPAD_UPDOWN and action == 2:
        # forward...
        if value == -32767:
            piwars2024.set_motor(255, 1, 255, 0)
        # reverse...
        if value == 32767:
            piwars2024.set_motor(255, 0, 255, 1)
        # stop...
        if value == 0:
            piwars2024.set_motor(0, 0, 0, 0)
    
    elif button == DPAD_LEFTRIGHT and action == 2:
        # left...
        if value == -32767:
            piwars2024.set_motor(255, 0, 255, 0)
        # right...
        if value == 32767:
            piwars2024.set_motor(255, 1, 255, 1)
        # stop...
        if value == 0:
            piwars2024.set_motor(0, 0, 0, 0)

    elif button == JOY1_UPDOWN:
        # value is -32767 to 32767
        # convert to -255 to 255
        speed = -int(value/32767*200)
        drive(speed, turning)
        #print("speed={}, turning={}".format(speed, turning))


    elif button == JOY2_LEFTRIGHT:
        # value is -32767 to 32767
        # convert to -255 to 255
        turning = int(value/32767*200)
        drive(speed, turning)
        #print("speed={}, turning={}".format(speed, turning))

    elif button == LEFT_TRIGGER:
        # value is -32767 to 32767
        # convert to 0 to 255
        
        val = int((value + 32767)*255/2/32767)
        piwars2024.set_servo(1, val)

    elif button == RIGHT_TRIGGER:
        # value is -32767 to 32767
        # convert to 0 to 255
        
        val = int((value + 32767)*255/2/32767)
        piwars2024.set_servo(2, val)

    #print(piwars2024.get_encoder())


