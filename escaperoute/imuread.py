import time
import datetime as dt
import piwars2024
import threading

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

class ImuReader(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.terminated = False

        style.use("fivethirtyeight")

        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1, 1, 1)
        self.xs = []
        self.ys = []
        self.id = 0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.start()

    def stop(self):
        self.terminated = True
        self.join()

    def run(self):
        # IMU capture thread...
        while True:
            xvec = []
            yvec = []
            count = piwars2024.get_fifo_count()
            #print("fifo_count = {}".format(count))
            while count > 0:
                imu = piwars2024.get_imu_float_fifo()
                #print("id = {}".format(imu[0]))
                #print("temp = {}".format(imu[1]))
                #print("accel = {}, {}, {}".format(imu[2], imu[3], imu[4]))
                #print("gyro = {}, {}, {}".format(imu[5], imu[6], imu[7]))
        
                self.roll = self.roll + imu[5] * 0.08
                self.pitch = self.pitch + imu[6] * 0.08
                self.yaw = self.yaw + imu[7] * 0.08

                xvec.append(imu[4])
                #yvec.append(dt.datetime.now().strftime("%H:%M:%S.%f"))
                yvec.append(self.id)
                self.id = self.id + 1
                count = piwars2024.get_fifo_count()
                #print("fifo_count = {}".format(count))
        
            self.xs.extend(xvec)
            self.ys.extend(yvec)
            time.sleep(0.25)

    def animate(self, i, xs, ys):
    
        #(newx, newy) = get_imu_data()
        #xs.extend(newx)
        #ys.extend(newy)

        xs = xs[-100:]
        ys = ys[-100:]

        #print("xs={}".format(xs))
        #print("ys={}".format(ys))

        self.ax1.clear()
        self.ax1.plot(ys, xs)

        plt.xticks(rotation=45, ha="right")
        plt.subplots_adjust(bottom=0.30,left=0.1)
        plt.title("IMU plot")
        plt.ylabel("value")

    def start_animation(self):
        ani = animation.FuncAnimation(self.fig, self.animate, fargs=(xs, ys), interval=200)
        plt.show()


