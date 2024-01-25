import asyncio
import evdev
from time import sleep

devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
for device in devices:
    print(device.path, device.name, device.phys)

gamepad = evdev.InputDevice('/dev/input/event2')

from motorControl import *
motor = motorControl(disable_motors=False)

print("doing...")

gamepad.grab()

#async def helper(gamepad):
#    async for ev in gamepad.async_read_loop():
#        print(repr(ev))
#loop = asyncio.get_event_loop()
#loop.run_until_complete(helper(gamepad))
#exit()

#for e in gamepad.read_loop():
while True:
    e = gamepad.read_one()
    if e is None:
        continue

    code = e.code
    value = e.value
    type = e.type
    if e.code != 0:
        print("----")
        print(f"code: {code}")
        print(f"type: {type}")
        print(f"value: {value}")

        if code == 16:
            if value == 1:
                # right
                motor.throttle(0, 1)
            elif value == -1:
                # left
                motor.throttle(1, 0)
            elif value == 0:
                # stop
                motor.throttle(0, 0)
                motor.release()

        if code == 17:
            if value == 1:
                # forward
                motor.throttle(1, 1)
            elif value == -1:
                # reverse
                motor.throttle(-1, -1)
            elif value == 0:
                # stop
                motor.throttle(0, 0)
                motor.release()

    else:
        print(e)
    sleep(0.01)


print("done")
