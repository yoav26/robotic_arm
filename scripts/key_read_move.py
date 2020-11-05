#!/usr/bin/env python

import math
import numpy as np
import rospy
import time
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from DirectKinmatics import sensor_location
from InverseKinmatics import joints_angles


DEG2RAD = math.pi / 180
POS2RAD = (2 * math.pi) / 4096
RAD2POS = 4096 / (2 * math.pi)
ZERO_OFFSET = 4096/2
joints_angles_rad = [0, 0, 0, 0, 0]
temp = [0, 0, 0, 0, 0]
angles_msg = Float32MultiArray()
location = Float32MultiArray()

q4 = [0, 0, 0, 0]
nw_angles_pub = rospy.Publisher("robotEndEffector/write/angles", Float32MultiArray, queue_size=10)


def joints_angle_read(data):
    global POS2RAD
    global ZERO_OFFSET
    global joints_angles_rad
    global temp
    global q4

    temp = data.data
    joints_angles_rad = list(data.data)


    joints_angles_rad[0] = (data.data[0] - ZERO_OFFSET) * POS2RAD   # <TODO> modulo or subtraction ?
    joints_angles_rad[1] = (data.data[1] - ZERO_OFFSET) * POS2RAD
    joints_angles_rad[2] = (data.data[2] - ZERO_OFFSET) * POS2RAD
    joints_angles_rad[3] = data.data[3] * DEG2RAD
    joints_angles_rad[4] = data.data[4] * DEG2RAD

    [a04, q4, q5] = sensor_location(joints_angles_rad[0], joints_angles_rad[1],
                                    joints_angles_rad[2], joints_angles_rad[3], joints_angles_rad[4])


def move(data):
    global q4
    new_angles_pos = list(temp)
    commands = data
    rospy.loginfo(commands)
    unit = 4
    pos = q4
    if commands.linear.z == 2.0:
        #   q4[2] = q4[2] + unit
        pos[2] = pos[2] + unit
    if commands.linear.z == -2.0:
        #   q4[2] = q4[2] - unit
        pos[2] = pos[2] - unit
    if commands.angular.x == -2.0:
        #   q4[0] = q4[0] - unit
        pos[0] = pos[0] - unit
    if commands.angular.x == 2.0:
        #   q4[0] = q4[0] + unit
        pos[0] = pos[0] + unit
    if commands.angular.y == -2.0:
        #   q4[1] = q4[1] - unit
        pos[1] = pos[1] - unit
    if commands.angular.y == 2.0:
        #   q4[1] = q4[1] + unit
        pos[1] = pos[1] + unit

    new_angles = joints_angles(pos[0], pos[1], pos[2])
    new_angles_pos[0] = new_angles[0] * RAD2POS + ZERO_OFFSET
    new_angles_pos[1] = new_angles[1] * RAD2POS + ZERO_OFFSET
    new_angles_pos[2] = new_angles[2] * RAD2POS + ZERO_OFFSET
    angles_msg.data = new_angles_pos
    nw_angles_pub.publish(angles_msg)
    rospy.loginfo(q4)

def key_read_move():

    rospy.init_node('key_read_move', anonymous=True)
    #pub1 = rospy.Publisher("write/angles", Float32MultiArray, queue_size=10)

    rospy.Subscriber('robotEndEffector/read/angles_read', Float32MultiArray, joints_angle_read)
    rospy.Subscriber('commands', Twist, move)

    a = 0
    while a <= 2:
        print('waiting {}'.format(a))
        print(joints_angles_rad)
        print(temp)
        time.sleep(1)
        a += 1

    rospy.spin()


if __name__ == '__main__':
    try:
        key_read_move()
    except rospy.ROSInterruptException:
        pass