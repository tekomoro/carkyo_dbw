#!/usr/bin/env python

import time
import Jetson.GPIO as GPIO
import os
import i2cdev

## PI I2C configrations
MCP4725_ADD_ACC = 0x61
MCP4725_ADD_BRAKE = 0x60
dac_acc = i2cdev.I2C(MCP4725_ADD_ACC, 8)
dac_brake = i2cdev.I2C(MCP4725_ADD_BRAKE, 8)

# Time Variables
mode_time  = time.time()




def accelerate_old(accVal):
    global accSwitch,  accPins,last_acc 
    last_acc = accVal       ### Just For Debuging
    dacValue = accVal # Range (0 to 4095)
    H = int(dacValue / 16)
    L = int((dacValue - (16*H)) * 16)
    data_to_send = bytes([64,H,L])
    dac_acc.write(data_to_send)
    #time.sleep(0.1)


def accelerate(accVal):
#    accVal *= 64 # for power 3.3v
    accVal *= 42 # for power 5v
    dacValue = accVal # Range (0 to 4095)
    if (dacValue > 4095):
        print ("dacValue error: " , dacValue)
        dacValue = 4085
    elif (dacValue < 10):
        print ("dacValue error: " , dacValue)
        dacValue = 10
    print("acc: ", dacValue)
    H = int(dacValue / 16)
    L = int(( dacValue - (16 * H) ) * 16)
    data_to_send = bytes([64,H,L])
    dac_acc.write(data_to_send)
    
    #time.sleep(0.1)


def brake(brakeVal):
    #brakeVal *= 64 # for power 3.3v
    brakeVal *= 42 # for power 5v       
    dacValue = brakeVal # Range (0 to 4095)
    if (dacValue > 4095):
        print ("dacValue error: " , dacValue)
        dacValue = 4085
    elif (dacValue < 10):
        print ("dacValue error: " , dacValue)
        dacValue = 10
    print("brake: ", dacValue)
    H = int(dacValue / 16)
    L = int(( dacValue - (16 * H) ) * 16)
    data_to_send = bytes([64,H,L])
    dac_brake.write(data_to_send)



for s in range(1000):
    for i in range(60):
        accelerate(i)
        brake(10)
        time.sleep(1)

