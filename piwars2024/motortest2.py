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

def makeint(val):
    if val < 32767:
        return val
    else:
        return -(65536-val)

def get_imu():
    cmd = [0xAB]
    spi.xfer2(cmd)
    len = spi.readbytes(1)[0]
    #print(len)
    if (len > 0):
        cs = spi.readbytes(len)
        #print(cs)

    t  = makeint((cs[0]<<8) + cs[1])
    ax = makeint((cs[2]<<8) + cs[3])
    ay = makeint((cs[4]<<8) + cs[5])
    az = makeint((cs[6]<<8) + cs[7])
    gx = makeint((cs[8]<<8) + cs[9])
    gy = makeint((cs[10]<<8) + cs[11])
    gz = makeint((cs[12]<<8) + cs[13])
    
    print("a=({},{},{})\ng=({},{},{})\nt={}".format(ax, ay, az, gx, gy, gz, t))

m = 0
dir = 1

while (True):

    m = m + dir
    if m == 255 or m == 0:
        dir = -dir

    set_motor(m, 1, m, 1)
    print("motor = {},{}".format(m, m))
    time.sleep(0.1)
    print("encoder = {}".format(get_encoder()))
    get_imu()



spi.close()

