#!/usr/bin/env python3
import math, serial
import time

#default_port='/dev/ttyTHS0'


#ard = serial.Serial(default_port,15200)#,timeout=0.0001)
ard = serial.Serial(
    port="/dev/ttyTHS0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
ard.flushInput()

while 1:
    #print (ard.inWaiting())
    if (ard.inWaiting() > 0):
        print (ard.inWaiting(), ard.read(ard.inWaiting()))
    #ard.write('H'.encode())
    #time.sleep(0.2)
