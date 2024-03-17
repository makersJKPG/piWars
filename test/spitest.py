import time
import spidev

bus = 0
device = 0
spi = spidev.SpiDev()
spi.open(bus, device)

spi.bits_per_word = 8
spi.max_speed_hz = 10000
spi.mode = 1

msg = [ 0xba, 0, 0, 0, 0 ]
print("Sending: {}".format(msg))
cs = spi.xfer2(msg)

b = spi.readbytes(5)
print("Received: {}".format(b))

#time.sleep(5)

spi.close()

