#!/usr/bin/env python3

from pygame import display, joystick, event
from pygame import QUIT, JOYAXISMOTION, JOYBALLMOTION, JOYHATMOTION, JOYBUTTONUP, JOYBUTTONDOWN

h = {
    (0,0):  'c',
    (1,0):  'E', (1,1):   'NE', (0,1):  'N', (-1,1): 'NW',
    (-1,0): 'W', (-1,-1): 'SW', (0,-1): 'S', (1,-1): 'SE'
}

P = 2 # precision

def main(argv=[]):
#    display.init()
    joystick.init()

    if len(argv) > 1:
        for sz in argv[1:]:
            try:
                joystick.Joystick(int(sz)).init()
            except:
                pass
    else:
        for i in range(joystick.get_count()):
            joystick.Joystick(i).init()

    e = event.wait()
    while e.type != QUIT:
        if e.type == JOYAXISMOTION:   print('js', e.joy, 'axis', e.axis, round(e.value, P))
        elif e.type == JOYBALLMOTION: print('js', e.joy, 'ball', e.ball, round(e.rel, P))
        elif e.type == JOYHATMOTION:  print('js', e.joy, 'hat', e.hat, h[e.value])
        elif e.type == JOYBUTTONUP:   print('js', e.joy, 'button', e.button, 'up')
        elif e.type == JOYBUTTONDOWN: print('js', e.joy, 'button', e.button, 'down')
        try:
            e = event.wait()
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    from sys import argv
    main(argv)

