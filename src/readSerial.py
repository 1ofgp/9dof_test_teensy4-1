import numpy as np
import serial
import argparse
import time
from time import gmtime, strftime, sleep 
from time import time_ns as ns


parser = argparse.ArgumentParser(description='Run info')
parser.add_argument('--port', dest='portName', type=str, default='/dev/ttyACM0',required=False)
parser.add_argument('--out', dest='out', type=str, default=('out-'+ time.strftime("%Y%m%d-%H%M%S") + '.txt'), help='output file',required=False)

args = parser.parse_args()

ser = serial.Serial()
print(args.portName)
ser.port = args.portName #Teensy serial port
ser.baudrate = 115200#9600
ser.timeout = 2000 #specify timeout when using readline()

ser.open()
# if ser.is_open==True:
#  	print("\nAll right, serial port now open. Configuration:\n")
#  	print(ser, "\n") #print serial parameters
ser.flushOutput()
ser.flushInput()

out_file = open(args.out, "a")

while ser.is_open == True:
    line=ser.readline().strip() #remove \r\n etc
    lineToFile = str(float(ns()/1.0e9)) + ' ' + line.decode()
    print(lineToFile)
    out_file.writelines(lineToFile + '\r\n')
   