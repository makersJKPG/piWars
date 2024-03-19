import struct

fmt = "ihBB"
infile_path = "/dev/input/js0"
EVENT_SIZE = struct.calcsize(fmt)
file = open(infile_path, "rb")
event = file.read(EVENT_SIZE)
while event:
    print("event type = {}, {}".format(type(event), len(event)))
    print(event)
    print(struct.unpack(fmt, event))
    (esec, etype, ecode, evalue) = struct.unpack(fmt, event)
    event = file.read(EVENT_SIZE)
