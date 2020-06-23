#!/usr/bin/env python3
import rospy, math, serial
import time
import numpy as np
import Jetson.GPIO as GPIO
import os
import i2cdev

MCP4725_ADD_ACC = 0x60
MCP4725_ADD_BRAKE = 0x61
#dac_acc = i2cdev.I2C(MCP4725_ADD_ACC, 8)
#dac_brake = i2cdev.I2C(MCP4725_ADD_BRAKE, 8)

#########
##Pins
#accSwitch   = 21
#modePins    = (16,22,18) # N - F - R
#
# i2c dac acc , brake -->  3 DAT , 5 CLK
#accPins     = (24,26,32,36,38,40)
#breakPins   = (23,29,31,33,35,37)
#########
from collections import deque
# ROS Msgs
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

#REGION SerialInit.
command ='udevadm info -e'
p=os.popen(command,"r")
data = p.read()
lines= data.split('\n\n')
default_port='/dev/ttyACM9'
default_port_eps='/dev/ttyACM10'

for i in range(len(lines)):
    lines[i].replace("P","\nP")
#    if 'Uno' in lines[i] and '855313036' in lines[i]:
    if 'Uno' in lines[i] and 'dev/ttyACM' in lines[i]:
        start = lines[i].find('/dev/ttyACM')
        if (start != -1):
            default_port = lines[i][start:start+12]
            print("Arduino port was found after search: ", default_port, "Srart was: ", start)

    if 'Uno' in lines[i] and '856333236' in lines[i]:
        start = lines[i].find('/dev/ttyACM')
        if (start != -1):
            default_port_eps = lines[i][start:start+12]
            print("Arduino port was found after search: ", default_port_eps, "Srart was: ", start)


#ard = serial.Serial(default_port,9600,timeout=0.0001)
#ard.flushInput()

#ENDREGION SerialInit.

logFile = open('/home/jetson/log.txt' , 'a+')
#ENDREGION MemoryFileHandling.



plan_on = False

DEBUG = 0

## PI GPIO configrations
GPIO.setmode(GPIO.BOARD)
accSwitch   = 21
modePins    = (16,22,18) #(16,18,22) # N - F - R
accPins     = (24,26,32,36,38,40)
breakPins   = (23,29,31,33,35,37)
GPIO.setup(modePins,GPIO.OUT)
GPIO.setup(accSwitch,GPIO.OUT)
GPIO.setup(accPins,GPIO.OUT)
GPIO.setup(breakPins,GPIO.OUT)
## End

## Global Variables
timeout_feedback_flag=0
command_angle=0.0
changing=0
started=0
kp     = 0.3 # PID control for speed
speed  = 0.0
v_mps  = 0.0
Vmps_arr   = [0.0]*4
angle_fb   = 0.0
last_acc   = 0
mean_Vmps  = 0.0
last_brake = 0
speed_counter = 0
key_speed = 0
frame = deque([])

# Time Variables
mode_time  = time.time()


rospy.init_node('main_Control_node', anonymous=True)

time.sleep(1)
last_command_time = rospy.Time.now()

#TODO : publish only one carkyo topic for raw sensors/actuators
#TODO : Do main pipeline in seperate loop other than callbacks
# Wheel Odometry Publisher
odom_pub  = rospy.Publisher('/wheel_odometry', Odometry, queue_size = 10)
odomMsg   = Odometry()
# Brake Value Publisher
brake_pub = rospy.Publisher('/brake', Int16, queue_size = 1)
brakeMsg  = Int16()
# steering angle Publisher
steering_pub = rospy.Publisher('pot_angle', AckermannDriveStamped, queue_size=10)
steering_msg = AckermannDriveStamped()


#REGION Serial
applyingBrakes = 0.0
in_start = 1
reading_done_count = 0
last_rf_state = 's'
tx = time.time()
pt = time.time()
dash_line_start = time.time()
countSteering = 0
def readSerial(event):
    global frame, reading_done_count, dash_line_start, countSteering
    global angle_fb,applyingBrakes, in_start , last_rf_state, tx, pt
    global plan_on
    global last_acc
    #print("")
    if (time.time()- dash_line_start > 30.0):
        print("--------------------------------------------------")
        dash_line_start = time.time()
    dt = time.time()-tx
    if (dt > 0.35):
        print("Delayed CB Call, Time Diff: ", dt)
    tx = time.time()
    errorCode = 0
    lastVal = 0.0
    pot_val = ""
    

    # read angle feedback from arduino
    reading_done = 0
    if in_start and (ard.inWaiting() > 0):
        ''' Flush
        Clear Buffer once at code start
        '''
        #temp_reader = ard.read(ard.inWaiting()) #HEMA 20191202
        ard.flushInput()
        in_start = 0
    
    '''
    if ard.inWaiting() > 9: #clearing all old messages before the newest 9
        ard.read(ard.inWaiting()-9)
        while( len(frame) > 0):
            #clear the entire frame if existing (No need for old data if new data >9)
            frame.popleft()
        print("clearing...")
    '''

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
        #print(frame)
        
        frame_clear_start = time.time()
        frame_length = len(frame)
        while(len(frame) > 9):
            frame.popleft()
        if (time.time()-frame_clear_start > 0.001):
            print("Clearing the oldest bytes of the frame took: ", time.time()-frame_clear_start, frame_length)
            
        if len(frame) == 5:
            #print("Case len (frame) = 5 : ", len(frame))
            if ( ord(frame[0]) == 255 and ord(frame[1]) == 255 and ord(frame[2]) <= 3):
                pot_val = ord(frame[2])*256 + ord(frame[3])
                rf_state = chr(ord(frame[4]))
                for i in range(5): #clearing the frame after reading
                    frame.popleft()
                    
                reading_done = 1

            elif ( ord(frame[3]) == 255 and ord(frame[4]) == 255 and ord(frame[0]) <= 3):
                pot_val = ord(frame[0])*256 + ord(frame[1])
                rf_state = chr(ord(frame[2]))
                for i in range(3): #clearing the frame after reading
                    frame.popleft()
                    
                reading_done = 1
                
            elif ( ord(frame[4]) == 255 and ord(frame[0]) == 255 and ord(frame[1]) <= 3):
                pot_val = ord(frame[1])*256 + ord(frame[2])
                rf_state = chr(ord(frame[3]))
                for i in range(4): #clearing the frame after reading
                    frame.popleft()
                    
                reading_done = 1
                
            else:  
                while(ord(frame[0]) != 255 or ord(frame[1]) != 255 or ord(frame[2]) > 3) and len(frame) > 3: #clearing all old messages before the header
                    frame.popleft()
                print("frame_len = 5")
                #reading_done is still 0

        elif len(frame) < 5: #leave the (<5) frames for next iteration 
            #print("frame_len < 5 , no good frame")
            pass # pass means keep reading_done = 0 

        elif len(frame)>=5:
            while(ord(frame[0]) != 255 or ord(frame[1]) != 255 or ord(frame[2]) > 3) and len(frame) > 3: #clearing all old messages before the header
                 frame.popleft()  

            if len(frame) < 5: 
                print("frame_len < 5 after being >5 and pop")
                pass # pass means keep reading_done = 0 
            else: 
                pot_val = ord(frame[2])*256 + ord(frame[3])
                rf_state = chr(ord(frame[4]))
                
                for i in range(5): #clearing the frame after reading
                    frame.popleft()
                    
                reading_done = 1
                  
        else:
            print("An unexpected buffer lenght {} >> skipping .. ".format(len(frame)))
            reading_done = 0
            


    if reading_done == 0: #NO new frame accepted
        print("No valid frame accepted ... skipping this iteration (0.0305) , No of last good frames is: ", reading_done_count)
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
            #print ('Safe')
            last_rf_state = 's'
        else:
            print ('Unknown Input')
            if (last_rf_state == 'e'):
                print ('Emergency')
                brakeMsg.data = 50
                brake_pub.publish(brakeMsg)
            errorCode = 1



        #TODO set the right equation
        zero_angle = 516.0
        slope =0.056916104137573
        low_pot_coeff = [-0.04303426 , 22.70881886]
        high_pot_coeff = [0.05340562 , -26.73804045]
        if pot_val > 20 and pot_val < 1020 :  # 50 and 974 is in the same distance from pot middle " 512 = (50+974)/2 "
            #new equation (2 lines)
            if pot_val > 518:
                angle_fb = - ( high_pot_coeff[0] * pot_val + high_pot_coeff[1] )
            elif pot_val < 509:
                angle_fb = low_pot_coeff[0] * pot_val + low_pot_coeff[1]
            else:
                angle_fb = 0.0


        else :
            '''
            Steering Angle Error From Arduino >> Apply Brakes here & stop stepper (through memory file sharing)
            '''
            print("Steering angle error ... pot is: ", pot_val)
            #angle_fb = 101.0 
            if plan_on:
                brakeMsg.data = 50
                brake_pub.publish(brakeMsg) 
            errorCode = 2

        #TODO we might instead publish this data in another published topic for all PI inputs & outputs to car/stepper ..  
            
        steering_msg.header.stamp=rospy.Time.now()
        steering_msg.drive.speed = pot_val
        steering_msg.drive.steering_angle = angle_fb
        steering_msg.drive.jerk =last_acc # errorCode #error code -> 2: wrong pot , 1: wrong serial input
        steering_pub.publish(steering_msg)
       
        steering_angle_deg = math.degrees(command_angle)
        angle_diff = steering_angle_deg - angle_fb
        if(angle_diff > 20):
            angle_diff = 20
        elif (angle_diff < -20):
            angle_diff = -20
        if (angle_diff > 0.3):
            voltage = translate(angle_diff, 0.0, 20.0, 2.1, 0.5)

        elif (angle_diff < -0.3):
            voltage = translate(angle_diff, 0.0, -20.0, 2.9, 4.5)
        else:
            voltage = 2.5
    
        #print("Volt: " , voltage , "Diff: ", angle_diff, "Angle FB ", angle_fb)
        if (voltage < 0.5 or voltage > 4.5):
             voltage = 2.5
        voltage = '{:02.2f}'.format(voltage)
        if applyingBrakes > 0.5 :  # HEMA Nov-27
            ard.write('e') 
            
            steering_msg.drive.jerk = 1
        else:
            ard.write('s') #s
            steering_msg.drive.jerk = 0

        if plan_on:
            ard.write(",a")
#            print("Automatic")
        else:
            ard.write(",m")
#            print("Manual")
        off_number = 6
        on_number = 11 - off_number
        if countSteering > off_number + on_number - 1 : # 7 off 4 on
            countSteering = 0
        elif countSteering > (on_number - 1) and abs(angle_diff) > 5:
            if (angle_diff > 0.0): #move to left
                steering_angle_deg = angle_fb + 0
            else :
                steering_angle_deg = angle_fb - 0
                
        countSteering += 1
        ard.write(","+str(steering_angle_deg)+"\n")
        pt = time.time() - tx
        if (pt > 0.03):
            print ("Delayed Processing Time: " , pt)

#ENDREGION Serial


#REGION: ModeFunctions
def set_mode_normal():
    global mode, modePins
    #print ("Setting mode normal ... ")
    modeVal = (0, 1, 1)
    brakeMsg.data = 60
    brake_pub.publish(brakeMsg)
    #applyBrake (60)
    GPIO.output(modePins, modeVal)
    mode = "Normal"


def set_mode_forward():
    global mode, modePins
    modeVal = (0, 0, 1)
    GPIO.output(modePins, modeVal)
    mode = "Forward"
    brake(10)


def set_mode_reverse():
    global mode, modePins
    modeVal = (0, 1, 0)
    GPIO.output(modePins, modeVal)
    mode = "Reverse"
    brake(10)


def set_mode_off():
    global mode, modePins
    modeVal = (1, 1, 1)
    GPIO.output(modePins, modeVal)
    mode = "Off"
#ENDREGION: ModeFunctions


#REGION: ControlFunctions
def accelerate(accVal):     ### input values: 10 ~ 63 
    global accSwitch,  accPins,last_acc , last_brake
    last_acc = accVal       ### Just For Debuging
    print ("Acc Val: ",accVal,"brake-val:" ,last_brake)
    if accVal <= 10 :
        GPIO.output(accSwitch, 1)
    else:
        GPIO.output(accSwitch, 0)
    #accVal = list("{0:b}".format(int(accVal)))
    #while len(accVal) < 6:
    #    accVal = ['0'] + accVal
    #accVal = map(int, accVal)
    #GPIO.output(accPins, accVal)

def brake(brakeVal):
    global last_brake
    
    last_brake = brakeVal   ### Just For Debuging
    #brakeVal = list("{0:b}".format(brakeVal))
    #while len(brakeVal) < 6:
    #    brakeVal = ['0'] + brakeVal
   # brakeVal = map(int, brakeVal)
   
    #GPIO.output(breakPins, brakeVal)
'''
def accelerate(accVal):
    global accSwitch,  accPins,last_acc , last_brake
    last_acc = accVal       ### Just For Debuging
    print ("Acc Val: ",accVal,"brake-val:" ,last_brake)
    
    if accVal <= 10 :
        GPIO.output(accSwitch, 1)
    else:
        GPIO.output(accSwitch, 0)
       
    dacValue = accVal # Range (0 to 4095)
    H = int(dacValue / 16)
    L = int(( dacValue - (16 * H) ) * 16)
    data_to_send = bytes([64,H,L])
    dac_acc.write(data_to_send)
    #time.sleep(0.1)

def brake(brakeVal):
    global last_brake
    last_brake = brakeVal   ### Just For Debuging
    
    dacValue = brakeVal # Range (0 to 4095)
    H = int(dacValue / 16)
    L = int(( dacValue - (16 * H) ) * 16)
    data_to_send = bytes([64,H,L])
    dac_brake.write(data_to_send)
    #time.sleep(0.1)
'''

def change_mode(newMode):
    global mode, changing, started
    if changing == 1:
        return
    if started == 0:
        changing = 1
        if DEBUG == 1: print ("Starting Mode: ", newMode)
        set_mode_off()
        time.sleep(2)
        accelerate(10)  # 0.6
        brake(10)       # 0.5 volts
        if newMode == "Normal":
            set_mode_normal()
            if DEBUG == 1: print ("Set Mode: ", newMode)
        elif newMode == "Forward":
            set_mode_forward()
        elif newMode == "Reverse":
            set_mode_reverse()
        elif newMode == "Off":
            set_mode_off()
        time.sleep(2.5)
        if DEBUG == 1: print("after")
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
lastCmdSpeed  = 0.0
jerk_response = 8       ### Just for debuging $ published im /wheel_odometry

def callback_move_base(data):
    global speed , Vmps_arr , kp , mean_Vmps , command_angle ,mode
    global mode_time, speed_counter, last_command_time
    global plan_on, lastCmdSpeed, jerk_response

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
        jerk_response = 10          ### Just for debuging $ published im /wheel_odometry
    else:       # No brakes needed, jerk = 0
        brakeMsg.data = 12
        brake_pub.publish(brakeMsg)
        #applyBrake(12)
    ## TODO: End
    
    lastCmdSpeed = data.drive.speed
    #if DEBUG == 1: print "Speed Callback = ",speed
    command_angle = data.drive.steering_angle
    if command_angle > 0.3316:
        command_angle = 0.3316
    elif command_angle < -0.4014:
        command_angle = - 0.4014

    ## Handling Mode Change
    if (speed >= -0.01) and (speed <= 0.01) and mode != "Normal" : ### Case 0
        change_mode("Normal")
        mode_time = time.time()
        if DEBUG == 1: print ("............Mode Changed to Normal : ",mode_time)
    elif speed < 0.0 and mode != "Reverse" :
        change_mode("Reverse")
        if DEBUG == 1: print ("............Mode Changed to Reverse : ",mode_time)
        mode_time = time.time()
    elif speed > 0.0 and mode != "Forward" :
        change_mode("Forward")
        if DEBUG == 1: print ("............Mode Changed to Forward : ",mode_time)
        mode_time = time.time()


# Main Function
last_mode="Normal"
zeroSpeedCounter=0
last_false_time =time.time()
tempFlag = 1
tempMode = ""
plan_end_key = 500
plan_key_changed = 0
#speedToDAC = 8     # convert from m/s to dac value
CalcDac = 0
lastVmps = 0

def callback_speed_feedback(data): #add odommsg as global
    global logFile
    global mode, last_mode, angle_fb, speed , v_mps, tempFlag, tempMode
    global speed_counter, Vmps_arr, command_angle, timeout_feedback_flag,last_command_time
    global plan_on,mean_Vmps,zeroSpeedCounter ,last_false_time ,jerk_response ,last_brake, last_acc
    global plan_end_key, plan_key_changed, CalcDac, lastVmps
    #if DEBUG == 1: print "Last_acc:",last_acc,"last_brake",last_brake,"Mode",mode,"last_mode",last_mode
    start_speed_feedback_pipeline = time.time()
    error = 0
    vL = 0.0
    vR = 0.0
    timeout_feedback_flag = 0
    left  = data.twist.twist.linear.x
    right = data.twist.twist.linear.y
    first_sensor  = data.twist.twist.angular.x
    second_sensor = data.twist.twist.angular.y
    
    if mode == "Reverse":
        tempFlag = 1
        if last_mode == "Forward":
	        vL = 0.0 
	        vR = 0.0
        else:
            vL        = -left
            vR        = -right
            vF_sensor = -first_sensor
            vS_sensor = -second_sensor
        last_mode = "Reverse"
    
    elif mode == "Forward":
        tempFlag = 1
        if last_mode == "Reverse":
            vL = 0.0  
            vR = 0.0
        else:
            vL        = left
            vR        = right
            vF_sensor = first_sensor
            vS_sensor = second_sensor
        last_mode = "Forward"
    
    elif mode == "Normal": ### Change to zero
        deltaT = time.time() - mode_time
        if tempFlag == 1:
            tempMode = last_mode
            tempFlag = 0
        if tempMode == "Forward" and deltaT < 1.0:
            if DEBUG == 1: print (" ============ Feedback Forward before Stopping")
            vL = left
            vR = right
            vF_sensor = first_sensor
            vS_sensor = second_sensor
        
        elif tempMode == "Reverse" and deltaT < 1.0:
            if DEBUG == 1: print (" ============ Feedback Backward before Stopping")
            vL = -left
            vR = -right
            vF_sensor = -first_sensor
            vS_sensor = -second_sensor
        
        else :
            if DEBUG == 1: print (" ============ Feedback Stopped")
            vL = 0.0
            vR = 0.0
            vF_sensor = 0.0
            vS_sensor = 0.0
   
    #TODO deal better with wrong vel encoder val

    if vL > 500 or vR > 500:
        v_mps = lastVmps
    else:
        v_mps = (vR + vL)/ 2.0
    lastVmps = v_mps
    ## Add this for mean v_mps
    Vmps_arr[speed_counter] = v_mps
    speed_counter=speed_counter+1
    if speed_counter>3 :
        speed_counter=0
    wheelAngle = angle_fb*math.pi/180.0
    
    odomMsg.twist.twist.linear.x = vR
    odomMsg.twist.twist.linear.y = vL
    odomMsg.twist.twist.linear.z = wheelAngle
    odomMsg.twist.twist.angular.x = last_acc
    odomMsg.twist.twist.angular.z = last_brake
    if jerk_response > 0.1:
        odomMsg.twist.twist.angular.y = jerk_response
        jerk_response = -10
    else:
        odomMsg.twist.twist.angular.y = 0.0
    odomMsg.header.frame_id = 'odom'
    odomMsg.child_frame_id  = 'base_footprint'
    odomMsg.header.stamp = rospy.Time.now()
    odom_pub.publish(odomMsg)
    

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
    
    if mode == "Reverse" and abs(speed) < 0.95 :
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
    needed_angle_change = command_angle - (angle_fb * math.pi/180.0)
    if (abs(needed_angle_change) > 0.35) :
        needed_angle_change = 0.35
    wait_parameter = translate(abs(needed_angle_change), 0.0 , 0.35 , 1.0, 0.0)
    if PIDspeed < 0.5 and abs(needed_angle_change) > 0.22:
        wait_parameter = 0.1
    PIDspeed = wait_parameter * PIDspeed
    #ENDREGION: AngleChange
    
    if (PIDspeed > 0.5):
        DACvalue = 4 * math.sqrt(70 * PIDspeed) + 4.5/PIDspeed
#    elif (PIDspeed <= 0.7 and PIDspeed >= 0.01):
    elif (PIDspeed <= 0.5 and PIDspeed >= 0.01):
        DACvalue = 20 * PIDspeed + 25
        #DACvalue = 20 * PIDspeed + 16
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
        plan_on=True

        # check the reason of bad control behaviour (acceleating / declerating !!)
        
        logLinePrint = "speed: %2.3f, pid: %2.3f, dac: %2.1f, vmps3: %2.3f, vr: %2.3f, vl: %2.3f, ticksL: %2.3f, ticksR: %2.3f, angNeededCh: %2.3f, error: %2.3f, brake: %2.1f\n" %(speed, PIDspeed, DACvalue, mean_Vmps, vR, vL, first_sensor, second_sensor, needed_angle_change, error, last_brake)
        logLine = "%2.3f,%2.3f,%2.1f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.1f\n" %(speed, PIDspeed, DACvalue, mean_Vmps, vR, vL, first_sensor, second_sensor, needed_angle_change, error, last_brake)
        #print(logLinePrint)  
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
#rospy.Timer(rospy.Duration((0.033)), readSerial, oneshot=False) # 0.0305 to sync with arduino serial communication
change_mode("Normal")

if __name__ == '__main__':
    listener()
 

