import time
import spidev

bus = 0
device = 0
spi = spidev.SpiDev()
spi.open(bus, device)

spi.bits_per_word = 8
spi.max_speed_hz = 10000
spi.mode = 1


while (True):
    # send get_log command
    cmd = [0xAA]
    spi.xfer2(cmd)

    # first byte is length available to read
    len = spi.readbytes(1)[0]
    #print("len: {}".format(len))
    
    if (len > 0):
        # read all characters
        chars = spi.readbytes(len)
        #print(chars)
        str = ""
        for c in chars:
            str += chr(c)
        print(str, end='')

        time.sleep(0.1)

spi.close()

