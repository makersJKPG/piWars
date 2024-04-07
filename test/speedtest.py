import piwars2024
import time

def accelerate(startspeed, endspeed, steps=10, intervaltime=0.1):
    
    stepchange = int((endspeed-startspeed) / steps)

    for i in range(0, steps+1):
        speed = startspeed + i*stepchange
        piwars2024.set_speed(speed, speed)
        print("speed={}".format(speed))
        time.sleep(intervaltime)


piwars2024.set_motor(0, 0, 0, 0)
piwars2024.set_speed(0, 0)
piwars2024.set_mode(1)

time.sleep(5)

accelerate(0, -50)

time.sleep(1)

piwars2024.set_speed(0, 0)
piwars2024.set_mode(2)
piwars2024.set_motor(0, 0, 0, 0)

