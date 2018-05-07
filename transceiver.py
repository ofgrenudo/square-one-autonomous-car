import serial
import sys
import time

ser = serial.Serial('/dev/ttyACM0', 9600)
sys.argv[1]

print("Sending Data {}", sys.argv[1])
ser.write("{}\n".format(sys.argv[1]).encode())

