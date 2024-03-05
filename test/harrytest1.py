import time
import spidev

bus = 0
device = 0
spi = spidev.SpiDev()
spi.open(bus, device)

spi.bits_per_word = 8
spi.max_speed_hz = 10000
spi.mode = 1

def set_motor(m1, dir1, m2, dir2):
    if m1 > 255:
        m1 = 255
    if m1 < 0:
        m1 = 0
    if m2 > 255:
        m2 = 255
    if m2 < 0:
        m2 = 0
    cmd = [ 0xBA, m1, dir1, m2, dir2]
    spi.xfer2(cmd)

def get_encoder():
    cmd = [0xAC]
    spi.xfer2(cmd)
    len = spi.readbytes(1)[0]
    cs = spi.readbytes(len)
    value = (cs[0]<<24) + (cs[1]<<16) + (cs[2]<<8) + cs[3]
    return value

def get_imu():
    cmd = [0xAB]
    spi.xfer2(cmd)
    len = spi.readbytes(1)[0]
    print(len)
    if (len > 0):
        cs = spi.readbytes(len)
        print(cs)


while (True):
    set_motor(255, 0, 255, 1)
    print("motor = 255,255")
    time.sleep(10)
    print("encoder = {}".format(get_encoder()))
    get_imu()

    set_motor(0, 1, 0, 1)
    print("motor = 0,0")
    time.sleep(10)
    print("encoder = {}".format(get_encoder()))
    get_imu()


spi.close()

