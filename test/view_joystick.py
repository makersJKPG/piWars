import struct

infile_path = "/dev/input/js0"
EVENT_SIZE = struct.calcsize("iHBB")
file = open(infile_path, "rb")
event = file.read(EVENT_SIZE)
while event:
    print(struct.unpack("iHBB", event))
    (tv_sec, type, code, value) = struct.unpack("iHBB", event)
    event = file.read(EVENT_SIZE)
