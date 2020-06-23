#!/usr/bin/env python

# KZOH: Changed TEN_MICRO to HUNDRED_MICRO (and variables)
# KZOH: Changed function names Thread to: TimmedTasks, EncoderHundredMicroTask, etc.
# KZOH: Removed thread and add a call to TimmedTasks at main
# Add the following functions for Stepper:
#	- ResetStepperPos()
#	- stepper_change(i)

# Accepts motion commands and send it to HW
# Receives HW status: Wheel odom from HW and car controller
##ma7moudk23Oct : seprate mode pins out of switches pins .. Switches value depends on DAC value


# pins 7,11 --> 7 -> L , 11 -> R
#####$$$$$##### Includes #####$$$$$##### 
#####$$$$$##### Includes #####$$$$$##### 
import roslib  
import rospy
import string
import math
import numpy as np
import re
import tf
import time 
from nav_msgs.msg import Odometry

import time 
import Jetson.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
###############################################################
###Global values for encoders######
NumberOfInterrputsPorts = 2   # 4 I/Os
HUNDRED_MICRO = 0.0001 # temp = 1 sec (10.0 / 1000.0) / (1000.0)        # units in sec
HunderedMicros = 0
Millies = 0
Secs = 0
Minutes = 0
TimingError = 0
CorrectTiming = 0
WaitCounter = 0

ticksPerSpinL = 202 #52
ticksPerSpinR = 202
ticksPerSpinMotor = 1024 #936 #1000
diameter = 0.436
lengthBetweenTwoWheels = 1.01
DistancePerCountL = (math.pi * diameter) / ticksPerSpinL
DistancePerCountR = (math.pi * diameter) / ticksPerSpinR
DistancePerCountMotor = (math.pi * diameter) / ticksPerSpinMotor

Prev = [0,0,0,0]
consecutivezeros = [0,0,0,0]
consecutiveones = [0,0,0,0]
threshold = [2,2,1,1]
Falling = [0,0,0,0]
Rising = [0,0,0,0]
Transition = [0,0,0,0]


pin=[7,11,15,13]  # left - right - speed #[13,15,7,11]
GPIO.setup(pin,GPIO.IN)
#GPIO.setup(pin,GPIO.IN,pull_up_down=GPIO.PUD_UP)

OneMilliesCounter = 0

# 7 m/s = ticksPerSpin*
#for (math.pi * diameter)= 1.369m  >>> 52 ticks
#for                        7   m  >>> 52*(7/1.369)
# 7 m means 52*(7/1.369) ticks
# 7 m in a second mean >> 7/20 m in 1/20 second
# Max possible ticks for 7 m in 1 second  = 7 /  DistancePerCount
# Max possible ticks between 2 time stamps(1/20 s) =  7 / (20* DistancePerCount)  = 13.29

encoderfrequency=20
max_speed=7.0
maxPossibleTicksL = max_speed / (encoderfrequency* DistancePerCountL)
maxPossibleTicksR = max_speed / (encoderfrequency* DistancePerCountR)
# If the sensor can measure speed up to 7 m/s , it can have 13 ticks between 2 time stamps
#( so time of one tick = 1/(20*13) =0.00384 ->  3.8 ms

OldTime = 0.0
OldTicksL = 0.0
OldTicksR = 0.0
OldTicks1 = 0.0
OldTicks2 = 0.0

#angle_pub = rospy.Publisher('/odom/angle', Odometry, queue_size=10)
rospy.init_node('encoder_node', anonymous=True)
odom_pub = rospy.Publisher('/speed_values', Odometry, queue_size=10)
odomMsg = Odometry()

# <<<<<<<< HEMA : 16 oct to solve car jumps due to encoder issues
VL = 0.0
VR = 0.0
lastVL = 0.0
lastVR = 0.0
acc_limit  = 1.1
countL = 0
countR = 0
                # max acceptable Diff bet. vl,lastvl OR vr,lastvr  #
                # max noticed decellration was 0.5 m/s per cycle in right sensor which was more reliable than left sensor 
limit = 1
                # max acceptable Diff bet. Vl,Vr OR vl,lastvl OR vr,lastvr  #
                # max limit diff in normal case was 1.054                   #
                # diff (lastvl - vl) @ problem time was 6,8,14 and 16       #
                # diff (lastvr - vr) @ problem time was normal              #
                # so limit must be > 1.02 :: 1.5 for now                    #
                # 14 Apr 2019 was 1.5 changed to solve the wrong feedback in vy .. TODO: monitor

def HundredMilliesTask():
    global DistancePerCountL, DistancePerCountR
    global Falling ,OldTime, DistancePerCountMotor, OldTicksL, OldTicksR, OldTicks1, OldTicks2, lastVL, lastVR, VL, VR
    global countL, countR
    TicksL = Falling[0]
    TicksR = Falling[1]
    Ticks1 = Falling[2]
    Ticks2 = Falling[3]
    CurrentTime = time.time()
    DiffTime    = (CurrentTime - OldTime)

    lastVL = VL
    lastVR = VR
    
    VL = (TicksL-OldTicksL) * DistancePerCountL / DiffTime
    VR = (TicksR-OldTicksR) * DistancePerCountR / DiffTime
    V1 = (Ticks1-OldTicks1) * DistancePerCountMotor / DiffTime
    V2 = (Ticks2-OldTicks2) * DistancePerCountMotor / DiffTime

    if countL >= 3:
        lastVl = VL
        countL = 0

    if countR >= 3:
        lasVR = VR
        countR = 0
 
    if (VL - lastVL  > acc_limit): # problem happend here whan i tried it, not sure if it is the same for vr !
        VL = lastVL
        countL += 1
    else:
        countL = 0


    if (VR - lastVR  > acc_limit):
        VR = lastVR
        countR += 1
    else:
        countR = 0

    #publish Odometry message
    odomMsg.pose.pose.position.x = V1
    odomMsg.pose.pose.position.y = V2
    odomMsg.pose.pose.position.z = 0.0
    odomMsg.twist.twist.linear.x = VL
    odomMsg.twist.twist.linear.y = VR
    odomMsg.twist.twist.linear.z = 0.0  
    odomMsg.twist.twist.angular.x = TicksL-OldTicksL
    odomMsg.twist.twist.angular.y = TicksR-OldTicksR
    odomMsg.twist.twist.angular.z = 0.0  
    odomMsg.pose.pose.orientation.x = TicksL #L
    odomMsg.pose.pose.orientation.y = TicksR #R
    odomMsg.pose.pose.orientation.z = 0.0
    odomMsg.header.frame_id = 'odom'
    odomMsg.child_frame_id  = 'base_footprint'
    odomMsg.header.stamp    = rospy.Time.now()
    odom_pub.publish(odomMsg)


    OldTicksL = TicksL
    OldTicksR = TicksR
    OldTicks1 = Ticks1
    OldTicks2 = Ticks2
    OldTime = CurrentTime       # Ususally it should be 100 millies

def GetRising():
    global Rising
    return (Rising[0], Rising[1],Rising[2],Rising[3])

def GetFalling():
    global Falling
    return (Falling[0], Falling[1],Falling[2],Falling[3])

def GetTransitions():
    global Transitions
    return (Transition[0], Transition[1],Transition[2],Transition[3])

def HundredMicroTask():
    global HunderedMicros, Millies, Secs, Minutes
    HunderedMicros += 1
    if HunderedMicros >=10:
        HunderedMicros=0
        Millies += 1
#        OneMilliesTasks()
        if Millies % 100 == 0.0:
            HundredMilliesTask()
        if Millies >=1000:
            Millies=0
            Secs += 1
            if Secs >=60:	
                Secs=0
                Minutes += 1

def EncoderHundredMicroTask():
    global NumberOfInterrputsPorts, Prev, consecutivezeros, consecutiveones, threshold, Falling, Rising, Transition ,pin
    for i in range(NumberOfInterrputsPorts):
        if GPIO.input(pin[i])==0:
            consecutivezeros[i]+=1
            consecutiveones[i]=0
            if Prev[i] == 1 and consecutivezeros[i] >= threshold[i]:
                Prev[i] = 0
                Falling[i]+=1
                Transition[i]+=1
        else:
            consecutivezeros[i]=0
            consecutiveones[i]+=1
            if Prev[i] == 0 and consecutiveones[i] >= threshold[i]:
                Prev[i] = 1
                Rising[i]+=1
                Transition[i]+=1

def TimmedTasks():
    global TimingError, CorrectTiming, WaitCounter, TimeElapsed
    Start = time.time() #timeit.timeit()
    Next = Start
    Next = Next + HUNDRED_MICRO

    print "Timmes task start"

    #Port[1-n].tmpcount=0, .Rising=0, .Falling=0, .Transition=0, .Prev=0
    while True:
        Now = time.time() # timeit.timeit()
        Start = Now
        if (Now >= Next):	# Exceeds the time
            TimingError+=1
        else:
            CorrectTiming+=1
        while (Now < Next):
            WaitCounter +=1
            Now = time.time() #timeit.timeit()
        EncoderHundredMicroTask()
        HundredMicroTask()
        Next = Next + HUNDRED_MICRO
        End = time.time()
        TimeElapsed = End - Start

if __name__ == '__main__':
    TimmedTasks()
