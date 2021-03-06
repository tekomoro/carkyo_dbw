#!/usr/bin/env python3
import rospy, math, serial
import time
import numpy as np
import Jetson.GPIO as GPIO
import os
import i2cdev
from collections import deque
# ROS Msgs
from std_msgs.msg import Int16, String
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from carkyo_dbw.msg import carkyo_msg

###PINS###
#accSwitch = 21 #modePins = (16,22,18) # N - F - R
# i2c dac acc , brake -->  3 DAT , 5 CLK
#brakePins   = (23,29,31,33,35,37)
#########

#REGION SerialInit.
command ='udevadm info -e'
p=os.popen(command,"r")
data = p.read()
lines= data.split('\n\n')
default_port='/dev/ttyACM9'

for i in range(len(lines)):
    lines[i].replace("P","\nP")
#    if 'Uno' in lines[i] and '855313036' in lines[i]:
    if 'Uno' in lines[i] and 'dev/ttyACM' in lines[i]:
        start = lines[i].find('/dev/ttyACM')
        if (start != -1):
            default_port = lines[i][start:start+12]
            print("Arduino port was found after search: ", default_port, "Srart was: ", start)

ard = serial.Serial(default_port,9600,timeout=0.0001)
ard.flushInput()
#ENDREGION SerialInit.

logFile = open('/home/jetson/log.txt' , 'a+')

## PI GPIO configrations
GPIO.setmode(GPIO.BOARD)
accSwitch   = 35
modePins    = (29,31,33)  # N - F - R

GPIO.setup(modePins,GPIO.OUT)
GPIO.setup(accSwitch,GPIO.OUT)

## End

## PI I2C configrations
MCP4725_ADD_ACC = 0x60
MCP4725_ADD_BRAKE = 0x61
dac_acc = i2cdev.I2C(MCP4725_ADD_ACC, 8)
dac_brake = i2cdev.I2C(MCP4725_ADD_BRAKE, 8)
## End

#TODO : Do main pipeline in seperate loop other than callbacks
#REGION ROS publishers
# Brake Value Publisher
brake_pub = rospy.Publisher('/brake', Int16, queue_size = 1)
brakeMsg  = Int16()
# carkyo actuators & sensors Publisher
carkyo_pub = rospy.Publisher('carkyo_msg', carkyo_msg, queue_size=10)
carkyo_msg = carkyo_msg()
# steering angle Publisher #TODO remove this publisher after testing
steering_pub = rospy.Publisher('pot_angle', AckermannDriveStamped, queue_size=10)
steering_msg = AckermannDriveStamped()
# debug string Publisher
debug_carkyo_msg_pub = rospy.Publisher('debug_carkyo_msg', String, queue_size=10)
debug_carkyo_msg = String()

#ENDREGION Publishers

## Global Variables
plan_on = False
DEBUG = 0
command_angle=0.0
speed  = 0.0
v_mps  = 0.0

rospy.init_node('main_Control_node', anonymous=True)

time.sleep(1)
last_command_time = rospy.Time.now()

#REGION Serial
angle_fb   = 0.0
pot_val = 512
frame = deque([])
applyingBrakes = 0.0
in_start = 1
reading_done_count = 0
last_rf_state = 's'
tx = time.time()
pt = time.time()
dash_line_start = time.time()
countSteering = 0
eps_voltage = 2.5

FRAME_LENGTH = 7
def readSerial(event):
    global frame, reading_done_count, dash_line_start, countSteering
    global angle_fb, in_start , last_rf_state, tx, pt
    global pot_val, eps_voltage
    if (time.time()- dash_line_start > 30.0):
        print("--------------------------------------------------")
        dash_line_start = time.time()
    dt = time.time()-tx
    if (dt > 0.35):
        print("Delayed CB Call, Time Diff: ", dt)
    tx = time.time()
    errorCode = 0
    lastVal = 0.0

    # read angle feedback from arduino
    reading_done = 0
    if in_start and (ard.inWaiting() > 0):
        ''' Flush
        Clear Buffer once at code start
        '''
        ard.flushInput()
        in_start = 0

    if (ard.inWaiting() <= 0):
        print("No in waiting!!!  : ", ard.inWaiting())
    if (ard.inWaiting() > 0):
        reading_start = time.time()
        no_of_waiting_bytes = ard.inWaiting()
        new_data = ard.read(no_of_waiting_bytes)
        reading_time = time.time() - reading_start
        if reading_time > 0.002 :
            print("Reading time is : ", reading_time) 

        if (len(new_data) != no_of_waiting_bytes):
            print("Readind failed to read all requied bytes: waiting_bytes: {} , read_bytes: {}".format(len(new_data), no_of_waiting_bytes) )
        frame.extend(new_data)
        
        frame_clear_start = time.time()
        frame_length = len(frame)

        while(len(frame) > (FRAME_LENGTH + 4)):
            frame.popleft()
        if (time.time()-frame_clear_start > 0.001):
            print("Clearing the oldest bytes of the frame took: ", time.time()-frame_clear_start, frame_length)
            
        if len(frame) == FRAME_LENGTH:
            #print("Case len (frame) = FRAME_LENGTH : ", len(frame))
            if ( ord(chr(frame[0])) == 255 and ord(chr(frame[1])) == 255 and ord(chr(frame[2])) <= 3): #data in 2,3,4,5 #most frequent 
                pot_val = ord(chr(frame[2]))*256 + ord(chr(frame[3]))
                rf_state = chr(ord(chr(frame[4])))
                received_angle = ord(chr(frame[5]))
                eps_voltage = ord(chr(frame[6]))
                for i in range(FRAME_LENGTH): #clearing the frame after reading
                    frame.popleft()
                    
                reading_done = 1

            elif ( ord(chr(frame[4])) == 255 and ord(chr(frame[5])) == 255 and ord(chr(frame[0])) <= 3): #data in 0,1,2,3
                pot_val = ord(frame[0])*256 + ord(frame[1])
                rf_state = chr(ord(chr(frame[2])))
                received_angle = ord(chr(frame[3]))
                eps_voltage = ord(chr(frame[6]))
                for i in range(FRAME_LENGTH - 2): #clearing the frame after reading
                    frame.popleft()
                    
                reading_done = 1
              
            elif ( ord(chr(frame[5])) == 255 and ord(chr(frame[0])) == 255 and ord(chr(frame[1])) <= 3): #data in 1,2,3,4
                pot_val = ord(frame[1])*256 + ord(frame[2])
                rf_state = chr(ord(chr(frame[3])))
                received_angle = ord(chr(frame[4]))
                eps_voltage = ord(chr(frame[6]))
                for i in range(FRAME_LENGTH - 1): #clearing the frame after reading
                    frame.popleft()
                    
                reading_done = 1
                
            else:  
                # Clearing all old messages before a header
                while(ord(chr(frame[0])) != 255 or ord(fchr(rame[1])) != 255 or ord(chr(frame[2])) > 3) and len(frame) > 3: 
                    frame.popleft()
                #print("frame_len = FRAME_LENGTH")  #reading_done is still 0

        elif len(frame) < FRAME_LENGTH: #leave the (<5) frames for next iteration 
            #print("frame_len < FRAME_LENGTH , no good frame")
            pass # pass means keep reading_done = 0 

        elif len(frame) >= FRAME_LENGTH:
            while(ord(chr(frame[0])) != 255 or ord(chr(frame[1])) != 255 or ord(chr(frame[2])) > 3) and len(frame) > 3: #clearing all old messages before the header
                 frame.popleft()  

            if len(frame) < FRAME_LENGTH: 
                print("frame_len < FRAME_LENGTH after being > FRAME_LENGTH and pop")
                pass # pass means keep reading_done = 0 
            else: 
                pot_val = ord(chr(frame[2]))*256 + ord(chr(frame[3]))
                rf_state = chr(ord(chr(frame[4])))
                received_angle = ord(chr(frame[5]))
                eps_voltage = ord(chr(frame[6]))
                
                for i in range(FRAME_LENGTH): #clearing the frame after reading
                    frame.popleft()
                    
                reading_done = 1
                  
        else:
            print("An unexpected buffer lenght {} >> skipping .. ".format(len(frame)))
            reading_done = 0

           


    if reading_done == 0: #NO new frame accepted
        print("No valid frame accepted ... skipping this iteration (0.033) , No of last good frames is: ", reading_done_count)
        reading_done_count = 0
        return
        
    else:
        reading_done_count +=1
        if rf_state == 'e' or rf_state == 'E':
            print ('Emergency')
            brakeMsg.data = 50
            brake_pub.publish(brakeMsg)
            last_rf_state = 'e'
        elif rf_state == 's' or rf_state == 'S':
            last_rf_state = 's'
        else:
            print ('Unknown Input')
            if (last_rf_state == 'e'):
                print ('Emergency')
                brakeMsg.data = 50
                brake_pub.publish(brakeMsg)
            errorCode = 1

        if (received_angle > 125): #[0, 250] 
            received_angle -=250 #[-125, 125]
            
        received_angle = received_angle / 5.0 #[-25, 25] deg

        eps_voltage = eps_voltage / 50.0 #[-25, 25] deg

        if pot_val > 20 and pot_val < 1020 :
            angle_fb = received_angle

        else :
            '''
            Steering Angle Error From Arduino >> Apply Brakes here if in planning mode
            '''
            print("Steering angle error ... pot is: ", pot_val)
            #angle_fb = 101.0
            if plan_on:
                print("Applying brakes .. ")
                brakeMsg.data = 50
                brake_pub.publish(brakeMsg) 
            errorCode = 2

        steering_angle_deg = math.degrees(command_angle)
        angle_diff = steering_angle_deg - angle_fb
        if(angle_diff > 20):
            angle_diff = 20
        elif (angle_diff < -20):
            angle_diff = -20

        if applyingBrakes > 0.5 :  # HEMA Nov-27
            ard.write('e'.encode()) 
        else:
            ard.write('s'.encode())

        if plan_on:
            ard.write(",a".encode())
        else:
            ard.write(",m".encode())

        
        off_number = 6
        on_number = 11 - off_number
        if countSteering > off_number + on_number - 1 : # 7 off 4 on
            countSteering = 0
        # Prevent overshooting at speeds more than 0.5
        # it can be done for stopped state by removing the speed cond.
        elif countSteering > (on_number - 1) and abs(angle_diff) > 5 and abs(v_mps) > 0.5:
            if (angle_diff > 0.0): #move to left
                steering_angle_deg = angle_fb + 0
            else :
                steering_angle_deg = angle_fb - 0
                
        countSteering += 1
        str_steering_angle_deg = "{:.2f}".format(steering_angle_deg)

        ard.write(("," + str_steering_angle_deg + "\n").encode())
        
        # Write PIDs to Arduino
        #ard.write((",0.14,0.002,0.01\n").encode())
        pt = time.time() - tx
        if (pt > 0.03):
            print ("Delayed Processing Time: " , pt)

#ENDREGION Serial


#REGION: ModeFunctions
current_mode = ""
def set_mode_normal():
    global current_mode, modePins
    #print ("Setting mode normal ... ")
    modeVal = (0, 1, 1)
    brakeMsg.data = 60
    brake_pub.publish(brakeMsg)
    #applyBrake (60)
    GPIO.output(modePins, modeVal)
    current_mode = "Normal"


def set_mode_forward():
    global current_mode, modePins
    modeVal = (0, 0, 1)
    GPIO.output(modePins, modeVal)
    current_mode = "Forward"
    brake(10)


def set_mode_reverse():
    global current_mode, modePins
    modeVal = (0, 1, 0)
    GPIO.output(modePins, modeVal)
    current_mode = "Reverse"
    brake(10)


def set_mode_off():
    global current_mode, modePins
    modeVal = (1, 1, 1)
    GPIO.output(modePins, modeVal)
    current_mode = "Off"
#ENDREGION: ModeFunctions


#REGION: ControlFunctions


def accelerate(accVal):
    global accSwitch, last_acc, last_brake
    
    
    last_acc = accVal       ### Just For Debuging
    #print ("Acc Val: ",accVal,"brake-val:" ,last_brake)

    #Comment accVal check
    if accVal <= 10 :
        GPIO.output(accSwitch, 1)
    else:
        GPIO.output(accSwitch, 0)
    #'''
#    accVal *= 64 # for power 3.3v
    accVal *= 42 # for power 5v
    dacValue = accVal # Range (0 to 4095)
    if (dacValue > 4095):
        print ("dacValue error: " , dacValue)
        dacValue = 4085
    elif (dacValue < 10):
        print ("dacValue error: " , dacValue)
        dacValue = 10
    H = int(dacValue / 16)
    L = int(( dacValue - (16 * H) ) * 16)
    data_to_send = bytes([64,H,L])
    dac_acc.write(data_to_send)
    #'''
    #time.sleep(0.1)


def brake(brakeVal):
    global last_brake

    last_brake = brakeVal   ### Just For Debuging
    #'''
    #brakeVal *= 64 # for power 3.3v
    brakeVal *= 42 # for power 5v       
    dacValue = brakeVal # Range (0 to 4095)
    if (dacValue > 4095):
        print ("dacValue error: " , dacValue)
        dacValue = 4085
    elif (dacValue < 10):
        print ("dacValue error: " , dacValue)
        dacValue = 10
    
    H = int(dacValue / 16)
    L = int(( dacValue - (16 * H) ) * 16)
    data_to_send = bytes([64,H,L])
    dac_brake.write(data_to_send)
    #'''


changing = 0
started = 0
def change_mode(newMode):
    global changing, started
    if changing == 1:
        return
    if started == 0:
        changing = 1
        set_mode_off()
        time.sleep(2)
        accelerate(10)  # 0.6
        brake(10)       # 0.5 volts
        if newMode == "Normal":
            set_mode_normal()
        elif newMode == "Forward":
            set_mode_forward()
        elif newMode == "Reverse":
            set_mode_reverse()
        elif newMode == "Off":
            set_mode_off()
        time.sleep(2.5)

        accelerate(10)  # 0.5 volts
        time.sleep(4)
        if newMode == "Off":
            started = 0
        else:
            started = 1
        changing = 0
    else:
        if newMode == "Normal":
            set_mode_normal()
        elif newMode == "Forward":
            set_mode_forward()
        elif newMode == "Reverse":
            set_mode_reverse()
        elif newMode == "Off":
            set_mode_off()

min_brake_delay = 0.3 # A value in both pid.py and steering.py

# stay 1.3 sec in case of 60
# stay 0.6 sec in case of 20
def brake_cb(data):
    global applyingBrakes
    brake_val=data.data
    if brake_val == 12:
        applyingBrakes = 0
        brake(10)   # 0.5 v 
        return
    if brake_val < 14 :
        brake_val = 14
    if brake_val > 60 :
        brake_val = 60

    applyingBrakes = 1

    accelerate(13)  # TODO: Test for right sequance to avoid stopping
    
    if (brake_val == 60 ):
        print ("set mode normal cb " , time.time())
        for i in range(14,brake_val):
            brake(i)
            time.sleep(0.003)
            if DEBUG == 1: print ("Brakes Value: ", i)
        delay_val = brake_val*min_brake_delay/14.0
        if delay_val > 1:
            delay_val = 1
        time.sleep(delay_val)
        for i in range(brake_val,14,-1):
            brake(i)
            time.sleep(0.003)
        accelerate(13)
        #print ("done set mode normal cb " , time.time())
        return
    
    for i in range(14,brake_val):
        brake(i)
        time.sleep(0.02)
        if DEBUG == 1: print ("Brakes Value: ", i)
    delay_val = brake_val*min_brake_delay/14.0
    if delay_val > 0.8:
        delay_val = 0.8
    time.sleep(delay_val)
    for i in range(brake_val,14,-1):
        brake(i)
        time.sleep(0.01)
    accelerate(13)
#ENDREGION: ControlFunctions


# On Planning Callback
mode_time  = time.time()
lastCmdSpeed  = 0.0
def callback_move_base(data):
    global speed, command_angle
    global mode_time, last_command_time
    global plan_on, lastCmdSpeed

    last_command_time = rospy.Time.now()
    plan_on = True
    
    ## TODO: Remove
    speed = data.drive.speed     # setpoint
    if (speed > 0.0 and speed < 0.5):
        pass #speed = 0.5 
    if data.drive.jerk >= 0.54 and data.drive.jerk <= 2.01 :
        applied_brake_val = translate(data.drive.jerk, 0.55 , 2.0 , 14, 20)
        brakeMsg.data = int(applied_brake_val)
        brake_pub.publish(brakeMsg)
        #applyBrake(int(applied_brake_val))
    else:       # No brakes needed, jerk = 0
        brakeMsg.data = 12
        brake_pub.publish(brakeMsg)
        #applyBrake(12)
    ## TODO: End
    
    lastCmdSpeed = data.drive.speed
    #if DEBUG == 1: print "Speed Callback = ",speed
    command_angle = data.drive.steering_angle
    if command_angle > 0.3316:
        rospy.logwarn("command_angle: %0.2f is larger than the limit ", command_angle)
        command_angle = 0.3316
    elif command_angle < -0.4014:
        rospy.logwarn("command_angle: %0.2f is less than the limit ", command_angle)
        command_angle = - 0.4014

    ## Handling Mode Change
    if (speed >= -0.01) and (speed <= 0.01) and current_mode != "Normal" : ### Case 0
        change_mode("Normal")
        mode_time = time.time()
        if DEBUG == 1: print ("............Mode Changed to Normal : ",mode_time)
    elif speed < 0.0 and current_mode != "Reverse" :
        change_mode("Reverse")
        if DEBUG == 1: print ("............Mode Changed to Reverse : ",mode_time)
        mode_time = time.time()
    elif speed > 0.0 and current_mode != "Forward" :
        change_mode("Forward")
        if DEBUG == 1: print ("............Mode Changed to Forward : ",mode_time)
        mode_time = time.time()


# Main Function
last_mode="Normal"
zeroSpeedCounter = 0
Vmps_arr   = [0.0]*4
last_false_time =time.time()
tempFlag = 1
tempMode = ""
plan_end_key = 500
plan_key_changed = 0
#speedToDAC = 8     # convert from m/s to dac value
CalcDac = 0
lastVmps = 0
timeout_feedback_flag = 0
speed_counter = 0
def callback_speed_feedback(data):
    global logFile
    global last_mode, angle_fb, speed , v_mps, tempFlag, tempMode
    global speed_counter, Vmps_arr, command_angle, timeout_feedback_flag,last_command_time
    global plan_on, zeroSpeedCounter ,last_false_time, last_acc
    global plan_end_key, plan_key_changed, CalcDac, lastVmps
    #if DEBUG == 1: print "Last_acc:",last_acc,"last_brake",last_brake,"Mode",current_mode,"last_mode",last_mode
    start_speed_feedback_pipeline = time.time()
    error = 0
    vL = 0.0
    vR = 0.0
    timeout_feedback_flag = 0
    left  = data.twist.twist.linear.x
    right = data.twist.twist.linear.y
    
    if not plan_on:
        vL = left
        vR = right

    elif current_mode == "Reverse":
        tempFlag = 1
        if last_mode == "Forward":
	        vL = 0.0 
	        vR = 0.0
        else:
            vL        = -left
            vR        = -right
        last_mode = "Reverse"
    
    elif current_mode == "Forward":
        tempFlag = 1
        if last_mode == "Reverse":
            vL = 0.0  
            vR = 0.0
        else:
            vL        = left
            vR        = right
        last_mode = "Forward"
    
    elif current_mode == "Normal": ### Change to zero
        deltaT = time.time() - mode_time
        if tempFlag == 1:
            tempMode = last_mode
            tempFlag = 0
        if tempMode == "Forward" and deltaT < 1.0:
            if DEBUG == 1: print (" ============ Feedback Forward before Stopping")
            vL = left
            vR = right
        
        elif tempMode == "Reverse" and deltaT < 1.0:
            if DEBUG == 1: print (" ============ Feedback Backward before Stopping")
            vL = -left
            vR = -right
        
        else :
            if DEBUG == 1: print (" ============ Stopping Feedback value ")
            vL = 0.0
            vR = 0.0
   
    #TODO deal better with wrong vel encoder val
    if vL > 500 or vR > 500:
        v_mps = lastVmps
        rospy.err("vL or vR has a very high speed ... check encoder")
    else:
        v_mps = (vR + vL)/ 2.0
    lastVmps = v_mps
    ## Add this for mean v_mps
    Vmps_arr[speed_counter] = v_mps
    speed_counter=speed_counter+1
    if speed_counter>3 :
        speed_counter=0
    wheelAngle = math.radians(angle_fb)

    #new_message
    carkyo_msg.v_l = vL
    carkyo_msg.v_r = vR
    carkyo_msg.steering_angle = wheelAngle
    carkyo_msg.pot_value = pot_val    
    carkyo_msg.dac_acc = last_acc        
    carkyo_msg.dac_brake = last_brake   
    carkyo_msg.header.stamp = rospy.Time.now()
    carkyo_pub.publish(carkyo_msg)


    mean_Vmps = np.mean(Vmps_arr)
    # Restart sequance after brakes block
    if (abs(mean_Vmps) < 0.01) and abs(speed) > 0.01 and plan_on and (time.time() - last_false_time) > 1.0: #TODO watch this in planning start>> was 10
    # Car not moving and cmd not zero and in planning since 10 sec.
        zeroSpeedCounter += 1 # inc. every time
    else:
        zeroSpeedCounter = 0
    
    if zeroSpeedCounter >= 10: # 25        
        accelerate(10) # 0.52 v
        time.sleep(0.2) # wait until car controller read the switch and pedal value
        zeroSpeedCounter = 0
    
    if current_mode == "Reverse" and abs(speed) < 0.95 :
        #if DEBUG == 1: print ("reverse speed" , speed)
        speed  = -0.95
    
    #REGION: PID
    kp = 0.3
    error = abs(speed) - abs(mean_Vmps)
    P = error*kp
    PIDspeed = P + abs(mean_Vmps)/2.0 + abs(speed) #+ PD    # This equation isn't OK in speeds more than 2.0 .. will overcome this problem as a temporary solution by limiting DAC value to 59 
    
    # apply PID speed constraints to equation output   
    if (PIDspeed > 1.25*abs(speed)):
        PIDspeed = 1.25*abs(speed)
    if (PIDspeed > abs(speed)+0.3):
        PIDspeed = abs(speed)+0.3

    #REGION: AngleChange
    '''
    Deals with speeds during different angle change values
    calculated in [wait_parameter] variable
    e.g.: lower speed when needed angle change is large
    '''
    needed_angle_change = command_angle - math.radians(angle_fb)
    if (abs(needed_angle_change) > 0.35) :
        needed_angle_change = 0.35
    elif (abs(needed_angle_change) < 0.1) :
        needed_angle_change = 0.1
    wait_parameter = translate(abs(needed_angle_change), 0.1 , 0.35 , 1.0, 0.0)
    '''
    #@low speed wait (stop) till angle is OK Test34Feb2020 remove it after replacing stepper with eps
    if PIDspeed < 0.5 and abs(needed_angle_change) > 0.22:
        wait_parameter = 0.1
    '''
    PIDspeed = wait_parameter * PIDspeed
    #ENDREGION: AngleChange
    
    if (PIDspeed > 0.5):
        DACvalue = 4 * math.sqrt(70 * PIDspeed) + 4.5/PIDspeed
    elif (PIDspeed <= 0.5 and PIDspeed >= 0.01):
        DACvalue = 20 * PIDspeed + 25
    elif (PIDspeed <= 0.01):
        DACvalue = 11
    #ENDREGION: PID
    
    if (rospy.Time.now()-last_command_time>=rospy.Duration(1.0)):
        '''
        if command not recieved for 1 sec
        '''
        if DEBUG == 1: print ("delayed command from feedback")
        accelerate(11)
        if plan_on: #TODO check if brake 20 for 1 iteration is enough
            brakeMsg.data = 20
            brake_pub.publish(brakeMsg)
        else:
            brakeMsg.data = 12
            brake_pub.publish(brakeMsg)
            #applyBrake(12)
        
        plan_on = False
        last_false_time=time.time()
 
    else:
        plan_on = True

        # check the reason of bad control behaviour (acceleating / declerating !!)

        logLinePrint = "eps_voltage: %2.2f, speed: %2.3f, pid: %2.3f, dac: %2.1f, vmps3: %2.3f, vr: %2.3f, vl: %2.3f, angNeededCh: %2.3f, error: %2.3f, brake: %2.1f\n" %(eps_voltage, speed, PIDspeed, DACvalue, mean_Vmps, vR, vL, needed_angle_change, error, last_brake)
        logLine = "%2.2f,%2.3f,%2.3f,%2.1f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.1f\n" %(eps_voltage, speed, PIDspeed, DACvalue, mean_Vmps, vR, vL, needed_angle_change, error, last_brake)
        print(logLinePrint)  
        debug_carkyo_msg.data = logLinePrint
        debug_carkyo_msg_pub.publish(debug_carkyo_msg) 
        #logFile.write(logLitne)

        if error < -0.3:
            DACvalue = 11
        if needed_angle_change > 0.1 : # 5.7 deg
            DACvalue = 11
        if (DACvalue > 68):
            print (" ** Large DAC Value >> Larger than expected max.", DACvalue)
        elif (DACvalue > 60):
            #print (" ** DAC Value larger than 60: ", DACvalue)
            DACvalue = 60
        if (int(DACvalue) <= 60 and int(DACvalue) > 9) : 
            #print ("Accelerating with {}".format(DACvalue))
            accelerate(int(DACvalue))
            #print (DACvalue)
            rospy.set_param("DACValue",int(DACvalue))

    if(time.time() - start_speed_feedback_pipeline > 0.1 ):
        print ("Time of speedFeedback pipeline: " , time.time() - start_speed_feedback_pipeline )

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

## Timout Function
def timeout_fun(event):
    '''
    Time-based fun. to Monitor Speed Feedback execution, take action if delayed
    '''
    global timeout_feedback_flag
    if (timeout_feedback_flag == 0 ):
        timeout_feedback_flag = 1
    else:
        print( "delayed feedback, disabling move base last command")
        accelerate(13)

# Listener
def listener():
    rospy.Subscriber("/rbcar_robot_control/command", AckermannDriveStamped, callback_move_base, queue_size=2)
    rospy.Subscriber("/speed_values", Odometry, callback_speed_feedback, queue_size=1)
    rospy.Subscriber("/brake", Int16, brake_cb, queue_size=2)
    rospy.spin()

rospy.Timer(rospy.Duration(1.0), timeout_fun)
rospy.Timer(rospy.Duration((0.033)), readSerial, oneshot=False) # 0.0305 to sync with arduino serial communication
change_mode("Normal")

if __name__ == '__main__':
    listener()
