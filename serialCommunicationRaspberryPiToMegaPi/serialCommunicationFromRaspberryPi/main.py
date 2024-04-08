import serial
import time

ser = serial.Serial('/dev/ttyS0',9600);

def read():
    while ser.in_waiting:
        data = ser.read()
        value = data.decode()
        print(value,end="")
        time.sleep(.1)

def write(string):
    ser.write((string + '\n').encode())
