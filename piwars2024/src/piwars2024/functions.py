import time
import spidev
import struct

debug = False

bus = 0
device = 0
spi = spidev.SpiDev()
spi.open(bus, device)

spi.bits_per_word = 8
spi.max_speed_hz = 10000
spi.mode = 1

def makeint32(val):
    if val < 32767:
        return val
    else:
        return -(65536-val)

def send_spi_cmd(cmd):
    sendlist = [ 0xa5 ]
    for i, val in enumerate(cmd):
        if val == 0xa5:
            sendlist.append(0xa4)
            sendlist.append(0x01)
        elif val == 0xa4:
            sendlist.append(0xa4)
            sendlist.append(0x00)
        else:
            sendlist.append(val)

    print("sending: {}".format(sendlist))
    cs = spi.xfer2(sendlist)
    print("received: {}".format(cs))

def set_motor(m1, dir1, m2, dir2):
    #print("set_motor({}, {}, {}, {})".format(m1, dir1, m2, dir2))
    if m1 > 255:
        m1 = 255
    if m1 < 0:
        m1 = 0
    if m2 > 255:
        m2 = 255
    if m2 < 0:
        m2 = 0
    cmd = [ 0xBA, m1, dir1, m2, dir2 ]
    send_spi_cmd(cmd)
    #print("sending: {}".format(cmd))
    #cs = spi.xfer2(cmd)
    #print("received: {}".format(cs))

def get_fifo_count():
    global debug
    if debug:
        print("get_fifo_count()")
    cmd = [0xAF, 0]
    if debug:
        print("sending: {}".format(cmd)) 
    cs = spi.xfer2(cmd)
    if debug:
        print("received: {}".format(cs))
    # note the byte order here, two 16 bit words in little endian format
    return cs[1]

def get_encoder():
    print("get_encoder()")
    cmd = [0xAC, 0, 0, 0, 0, 0, 0, 0, 0]
    #print("sending: {}".format(cmd)) 
    cs = spi.xfer2(cmd)
    #print("received: {}".format(cs))
    # note the byte order here, two 16 bit words in little endian format
    value1 = (cs[2]<<24) + (cs[1]<<16) + (cs[4]<<8) + cs[3]
    value1 = struct.unpack("i", value1.to_bytes(4, "little"))[0]
    value2 = (cs[6]<<24) + (cs[5]<<16) + (cs[8]<<8) + cs[7]
    value2 = struct.unpack("i", value2.to_bytes(4, "little"))[0]
    return value1, value2

def get_imu():
    print("get_imu()")
    cmd = [0xAB, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    #print("sending: {}".format(cmd)) 
    cs = spi.xfer2(cmd)
    #print("received: {}".format(cs))
    t  = (cs[1]<<8) + cs[2]; 
    ax = makeint32((cs[3]<<8) + cs[4]); 
    ay = makeint32((cs[5]<<8) + cs[6]); 
    az = makeint32((cs[7]<<8) + cs[8]); 
    gx = makeint32((cs[9]<<8) + cs[10]); 
    gy = makeint32((cs[11]<<8) + cs[12]); 
    gz = makeint32((cs[13]<<8) + cs[14]); 
    return (t, ax, ay, az, gx, gy, gz)

def get_imu_float():
    print("get_imu()")
    cmd = [0xAD, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    #print("sending: {}".format(cmd)) 
    cs = spi.xfer2(cmd)
    #print("received: {}".format(cs))

    cs.pop(0)   # remove first byte
    ba = bytearray(cs)

    imudata = struct.unpack("<fffffff", ba)
    return imudata

def get_imu_float_fifo():
    global debug
    if debug:
        print("get_imu_fifo()")
    cmd = [0xAE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    if debug:
        print("sending: {}".format(cmd)) 
    cs = spi.xfer2(cmd)
    if debug:
        print("received: {}".format(cs))

    cs.pop(0)   # remove first byte
    ba = bytearray(cs)

    imudata = struct.unpack("<Hfffffff", ba)
    return imudata


def shutdown():
    spi.close()

