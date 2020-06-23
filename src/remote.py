#!/usr/bin/env python

import rospy, sys
from ackermann_msgs.msg import AckermannDriveStamped
from carkyo_dbw.msg import carkyo_msg


import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

cmd = ''
isNew = False

def onConnect(client, userdata, flags, rc):
    print ("Connected to server - Code: "+ str(rc))
    client.subscribe('carkyo/cmd')

def onMessage(client, userdate, msg):
    global cmd, isNew
    #lastCmd = cmd
    cmd = str(msg.payload)
    #isNew = (cmd != lastCmd)
    #if (isNew):
    update()

client = mqtt.Client()
client.on_connect = onConnect
client.on_message = onMessage

speed = 0
steering = 0

speedInc = 0.1
steeringInc = 0.05

maxSpeed = 3.0
maxSteering = 0.40

speedFb = 0
steeringFb = 0

def update():
    global speed, steering
    if (cmd == 's'):
        steering = 0
        speed = 0
    elif (cmd == 'f'):
        speed += speedInc
    elif (cmd == 'b'):
        speed -= speedInc
    elif (cmd == 'r'):
        steering -= steeringInc
    elif (cmd == 'l'):
        steering += steeringInc
    elif(cmd == 'a'):
        steering = maxSteering
    elif(cmd == 'd'):
        steering = -maxSteering
    
    # Check Speed Limits
    if (speed > maxSpeed):
        speed = maxSpeed
    elif (speed < -maxSpeed):
        speed = -maxSpeed
    
    # Check Steering Limits
    if (steering > maxSteering):
        steering = maxSteering
    elif (steering < -maxSteering):
        steering = -maxSteering
    
    sys.stderr.write('\x1b[2J\x1b[H')
    rospy.loginfo('\x1b[1M\r'
                  '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                  '\033[34;1mSteer Angle: \033[32;1m%0.2f rad\033[0m',
                  speed, steering)
    
    # ROS Publish
    ackermann_cmd_msg = AckermannDriveStamped()
    ackermann_cmd_msg.header.stamp = rospy.Time.now()
    ackermann_cmd_msg.drive.speed  = speed
    ackermann_cmd_msg.drive.steering_angle = steering
    ackermann_cmd_msg.drive.jerk = 0
    cmd_pub.publish(ackermann_cmd_msg)
    
    # MQTT Publish
    msg = '{:0.2f},{:0.2f},{:0.2f},{:0.2f}'.format(speed,
                                                   steering,
                                                   speedFb,
                                                   steeringFb)
    publish.single('carkyo/jetson/cmd', msg, hostname='broker.hivemq.com')

cmd_topic = '/keyboard'
cmd_pub = rospy.Publisher(cmd_topic, AckermannDriveStamped, queue_size=1)



def cb(data):
    global speedFb, steeringFb
    speedFb = (data.v_l + data.v_r) / 2.0
    steeringFb = data.steering_angle
    '''
    data.pot_value  
    data.dac_acc       
    data.dac_brake
    '''


def listener():
    rospy.Subscriber("/carkyo_msg", carkyo_msg, cb, queue_size=2)


if __name__ == '__main__':
    rospy.init_node('remote_drive_node')
    listener()
    client.connect('broker.hivemq.com', 1883, 60)
    client.loop_forever()
    
