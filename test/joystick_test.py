import piwars2024
import struct

infile_path = "/dev/input/js0"
event_size = struct.calcsize("LhBB")

file = open(infile_path, "rb", buffering=0)

while True:
        
    event = file.read(event_size)
    styring = struct.unpack("LhBB", event)
    
    milli, value, action, button = styring
    print("milli={} value={} action={} button={}".format(milli, value, action, button))
    
    if button == 7:
        # forward...
        if value == -32767:
            piwars2024.set_motor(150, 1, 150, 0)
        # reverse...
        if value == 32767:
            piwars2024.set_motor(150, 0, 150, 1)
        # stop...
        if value == 0:
            piwars2024.set_motor(0, 0, 0, 0)
    
    elif button == 6:
        # left...
        if value == -32767:
            piwars2024.set_motor(150, 1, 150, 1)
        # right...
        if value == 32767:
            piwars2024.set_motor(150, 0, 150, 0)
        # stop...
        if value == 0:
            piwars2024.set_motor(0, 0, 0, 0)
 
    #print(piwars2024.get_encoder())

