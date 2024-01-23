import time
import piwars2024

while (True):
    print("")
    #piwars2024.set_motor(55, 0, 55, 1)
    #time.sleep(1)
    #enc = piwars2024.get_encoder()
    #print("encoder = {}".format(enc))
    count = piwars2024.get_fifo_count()
    print("fifo_count = {}".format(count))
    while count > 0:
        imu = piwars2024.get_imu_float_fifo()
        print("id = {}".format(imu[0]))
        #print("temp = {}".format(imu[1]))
        #print("accel = {}, {}, {}".format(imu[2], imu[3], imu[4]))
        #print("gyro = {}, {}, {}".format(imu[5], imu[6], imu[7]))
        count = piwars2024.get_fifo_count()
        print("fifo_count = {}".format(count))
    
    time.sleep(0.05)


shutdown()

