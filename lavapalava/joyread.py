import struct
import math
import threading

class JoyReader(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.terminated = False
        self.eventlist = []
        self.start()

    def run(self):
        print("In joy reader thread...")
        pack_format = "ihBB"
        event_size = struct.calcsize(pack_format)
        file_path = "/dev/input/js0"

        with open(file_path, "rb", buffering=0) as file:

            while not self.terminated:
                event = file.read(event_size)
                milli, value, action, button = struct.unpack(pack_format, event)
                print("milli={} value={} action={} button={}".format(milli, value, action, button))
                if button == 7:
                    self.eventlist.append({ 
                        "milli": milli,
                        "value": value,
                        "action": action,
                        "button": button
                    })

    def events_available(self):
        return len(self.eventlist)

    def get_event(self):
        if len(self.eventlist) > 0:
            event = self.eventlist[0]
            self.eventlist.pop()
            return event
        else:
            return None

