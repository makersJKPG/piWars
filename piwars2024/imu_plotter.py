import time
import datetime as dt
import piwars2024
from threading import Thread

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use("fivethirtyeight")

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
xs = []
ys = []
id = 0

def imu_capture_thread():
    global xs, ys, id;
    while True:
        xvec = []
        yvec = []
        count = piwars2024.get_fifo_count()
        print("fifo_count = {}".format(count))
        while count > 0:
            imu = piwars2024.get_imu_float_fifo()
            print("id = {}".format(imu[0]))
            #print("temp = {}".format(imu[1]))
            #print("accel = {}, {}, {}".format(imu[2], imu[3], imu[4]))
            #print("gyro = {}, {}, {}".format(imu[5], imu[6], imu[7]))
        
            xvec.append(imu[4])
            #yvec.append(dt.datetime.now().strftime("%H:%M:%S.%f"))
            yvec.append(id)
            id = id + 1
            count = piwars2024.get_fifo_count()
            print("fifo_count = {}".format(count))
        
        xs.extend(xvec)
        ys.extend(yvec)
        time.sleep(0.25)

def animate(i, xs, ys):
    
    #(newx, newy) = get_imu_data()
    #xs.extend(newx)
    #ys.extend(newy)

    xs = xs[-100:]
    ys = ys[-100:]

    #print("xs={}".format(xs))
    #print("ys={}".format(ys))

    ax1.clear()
    ax1.plot(ys, xs)

    plt.xticks(rotation=45, ha="right")
    plt.subplots_adjust(bottom=0.30,left=0.1)
    plt.title("IMU plot")
    plt.ylabel("value")

imu_thread = Thread(target=imu_capture_thread)
imu_thread.start()


ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=200)
plt.show()

#shutdown()

