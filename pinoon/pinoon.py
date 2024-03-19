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

def drive(speed, turning):

    lval = abs(speed)
    rval = abs(speed)
    if speed > 0:
        ldir = 1
        rdir = 1
    else:
        ldir = 0
        rdir = 0

    angle = turning*math.pi/2/255
    scale = math.cos(angle)
    if angle > 0:
        lval = int(lval * scale)
    else:
        rval = int(rval * scale)

    piwars2024.set_motor(lval, ldir, rval, rdir)

turning = 0
speed = 0

while True:
        
    event = file.read(event_size)
    styring = struct.unpack(pack_format, event)
    
    milli, value, action, button = styring
    print("milli={} value={} action={} button={}".format(milli, value, action, button))
    
    if button == DPAD_UPDOWN:
        # forward...
        if value == -32767:
            piwars2024.set_motor(150, 1, 150, 0)
        # reverse...
        if value == 32767:
            piwars2024.set_motor(150, 0, 150, 1)
        # stop...
        if value == 0:
            piwars2024.set_motor(0, 0, 0, 0)
    
    elif button == DPAD_LEFTRIGHT:
        # left...
        if value == -32767:
            piwars2024.set_motor(150, 1, 150, 1)
        # right...
        if value == 32767:
            piwars2024.set_motor(150, 0, 150, 0)
        # stop...
        if value == 0:
            piwars2024.set_motor(0, 0, 0, 0)

    elif button == JOY1_UPDOWN:
        # value is -32767 to 32767
        # convert to -255 to 255
        speed = -int(value/32767*255)
        drive(speed, turning)

    elif button == JOY2_LEFTRIGHT:
        # value is -32767 to 32767
        # convert to -255 to 255
        turning = int(value/32767*255)
        drive(speed, turning)

    #print(piwars2024.get_encoder())


