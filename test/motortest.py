import time
import piwars2024

while (True):
    print("")
    piwars2024.set_motor(0xa4, 0, 0xa5, 0)
    time.sleep(3)
#    enc1, enc2 = piwars2024.get_encoder()
#    print("encoder = {},{}".format(enc1, enc2))
    #imu = piwars2024.get_imu()
    #print("imu = {}".format(imu))
#    imu = piwars2024.get_imu_float()
#    print("temp = {}".format(imu[0]))
#    print("accel = {}, {}, {}".format(imu[1], imu[2], imu[3]))
#    print("gyro = {}, {}, {}".format(imu[4], imu[5], imu[6]))
    
    piwars2024.set_motor(50, 1, 50, 1)
    time.sleep(3)
#    enc1, enc2 = piwars2024.get_encoder()
#    print("encoder = {},{}".format(enc1, enc2))
    #imu = piwars2024.get_imu()
    #print("imu = {}".format(imu))
#    imu = piwars2024.get_imu_float()
#    print("temp = {}".format(imu[0]))
#    print("accel = {}, {}, {}".format(imu[1], imu[2], imu[3]))
#    print("gyro = {}, {}, {}".format(imu[4], imu[5], imu[6]))
  
    print("")
    piwars2024.set_motor(0, 0, 0, 0)
    time.sleep(1)
#    enc1, enc2 = piwars2024.get_encoder()
#    print("encoder = {},{}".format(enc1, enc2))
    #imu = piwars2024.get_imu()
    #print("imu = {}".format(imu))
#    imu = piwars2024.get_imu_float()
#    print("temp = {}".format(imu[0]))
#    print("accel = {}, {}, {}".format(imu[1], imu[2], imu[3]))
#    print("gyro = {}, {}, {}".format(imu[4], imu[5], imu[6]))
 

shutdown()

