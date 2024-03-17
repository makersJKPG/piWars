import serial
from time import sleep

ser = serial.Serial ("/dev/ttyAMA0", 9600)    #Open port with baud rate
while True:
    #print("waiting for data")
    line = ser.readline()              #read serial port
    #print("got data")
    #sleep(0.03)
    #data_left = ser.inWaiting()             #check for remaining byte
    #received_data += ser.read(data_left)
    print(line.decode(), end="")                   #print received data
    #ser.write(received_data)                #transmit data serially 

