#!/usr/bin/env python3
import rospy, math, serial
import time
import os
from collections import deque
# ROS Msgs
from std_msgs.msg import Int16
from ackermann_msgs.msg import AckermannDriveStamped
steering_pub = rospy.Publisher('pot_angle', AckermannDriveStamped, queue_size=10)
steering_msg = AckermannDriveStamped()

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


#TODO : Do main pipeline in seperate loop other than callbacks
#REGION ROS publishers
# Brake Value Publisher
brake_pub = rospy.Publisher('/brake', Int16, queue_size = 1)
brakeMsg  = Int16()

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
FRAME_LENGTH = 6
def readSerial(event):
    global frame, reading_done_count, dash_line_start, countSteering
    global angle_fb, in_start , last_rf_state, tx, pt
    global pot_val
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
                for i in range(FRAME_LENGTH): #clearing the frame after reading
                    frame.popleft()
                    
                reading_done = 1

            elif ( ord(chr(frame[4])) == 255 and ord(chr(frame[5])) == 255 and ord(chr(frame[0])) <= 3): #data in 0,1,2,3
                pot_val = ord(frame[0])*256 + ord(frame[1])
                rf_state = chr(ord(chr(frame[2])))
                received_angle = ord(chr(frame[3]))
                for i in range(FRAME_LENGTH - 2): #clearing the frame after reading
                    frame.popleft()
                    
                reading_done = 1
              
            elif ( ord(chr(frame[5])) == 255 and ord(chr(frame[0])) == 255 and ord(chr(frame[1])) <= 3): #data in 1,2,3,4
                pot_val = ord(frame[1])*256 + ord(frame[2])
                rf_state = chr(ord(chr(frame[3])))
                received_angle = ord(chr(frame[4]))
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

        #TODO set the right equation
        #TODO remove this calculations and receive the already calculated angle from arduino
        if (received_angle > 125): #[0, 250] 
            received_angle -=250 #[-125, 125]
            
        received_angle = received_angle / 5.0 #[-25, 25] deg

        if pot_val > 20 and pot_val < 1020 :
            angle_fb = received_angle

        else :
            '''
            Steering Angle Error From Arduino >> Apply Brakes here if in planning mode
            '''
            print("Steering angle error ... pot is: ", pot_val)
            if plan_on:
                print("Applying brakes .. ")
                brakeMsg.data = 50
                brake_pub.publish(brakeMsg) 
            errorCode = 2


        #TODO Test then remove
           
        steering_msg.header.stamp=rospy.Time.now()
        steering_msg.drive.speed = pot_val
        steering_msg.drive.steering_angle = angle_fb
        steering_msg.drive.jerk = errorCode #error code -> 2: wrong pot , 1: wrong serial input
        steering_pub.publish(steering_msg)
        
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
        elif countSteering > (on_number - 1) and abs(angle_diff) > 5 and abs(v_mps) > 0.5:
            if (angle_diff > 0.0): #move to left
                steering_angle_deg = angle_fb + 0
            else :
                steering_angle_deg = angle_fb - 0
                
        countSteering += 1
        steering_angle_deg = "{:.2f}".format(steering_angle_deg)

        ard.write(("," + steering_angle_deg + "\n").encode())
        pt = time.time() - tx
        if (pt > 0.03):
            print ("Delayed Processing Time: " , pt)

#ENDREGION Serial

# Listener
def listener():
    rospy.spin()

rospy.Timer(rospy.Duration((0.033)), readSerial, oneshot=False) # 0.0305 to sync with arduino serial communication

if __name__ == '__main__':
    listener()
