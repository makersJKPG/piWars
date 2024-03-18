import time
import spidev

bus = 0
device = 0
spi = spidev.SpiDev()
spi.open(bus, device)
spi.bits_per_word = 8
spi.max_speed_hz = 10000
spi.mode = 1
spi.read0 = False
spi.cshigh = False


#msg = [ 0xba, 0, 0, 0, 0]
msg = [ 20 ] 
print("Sending: {}".format(msg))
cs = spi.xfer2(msg)
print("Received: {}".format(cs))

spi.close()

