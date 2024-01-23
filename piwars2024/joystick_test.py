import piwars2024
import struct

infile_path = "/dev/input/js0"
EVENT_SIZE = struct.calcsize("LhBB")
file = open(infile_path, "rb")
event = file.read(EVENT_SIZE)
styring = struct.unpack("LhBB", event)
while event:
        print(styring)
        #(tv_msec, value, type, number) = struct.unpack("LhBB", event)
        event = file.read(EVENT_SIZE)
        styring = struct.unpack("LhBB", event)
        piwars2024.set_motor(0,0,0,1)
        if styring[1] == -32767 and styring[-1] == 7:
            piwars2024.set_motor(55,0,55,1)
        if styring[1] == 32767 and styring[-1] == 7:
            piwars2024.set_motor(55,1,55,0)
        if styring[1] == -32767 and styring[-1] == 6:
            piwars2024.set_motor(55,1,55,1)
        if styring[1] == 32767 and styring[-1] == 6:
            piwars2024.set_motor(55,0,55,0)

        print(piwars2024.get_encoder())
