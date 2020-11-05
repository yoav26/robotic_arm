#!/usr/bin/env python

import math
import numpy as np
import rospy
import time
from std_msgs.msg import Float32MultiArray, Float32
from InverseKinmatics import joints_angles

''' this scan node responsible for the automated scanning process
    reads current location and surface orientation, calculates the path and send the commends to the servo motors
    * the rotation refers the correction of the rotation to get a straight path on tilted surfaces '''


DEG2RAD = math.pi / 180
POS2RAD = (2 * math.pi) / 4096
RAD2POS = 4096 / (2 * math.pi)
ZERO_OFFSET = 4096/2
joints_angles_rad = [0, 0, 0, 0, 0]
l4 = 31
delta = 1
temp = [0, 0, 0, 0, 0]
angles_msg = Float32MultiArray()
location = Float32MultiArray()

pub_location = rospy.Publisher("location", Float32MultiArray, queue_size=10)
#pub1 = rospy.Publisher("write/angles", Float32MultiArray, queue_size=10)


def joints_angle_read(data):
    global POS2RAD
    global ZERO_OFFSET
    global joints_angles_rad
    global temp

    temp = data.data
    joints_angles_rad = list(data.data)

    #rospy.loginfo(rospy.get_caller_id() + ' \n --Got the joints angles%%%%%%: %f %f %f %f %f', data.data[0], data.data[1], data.data[2], data.data[3], data.data[4])
    # joints_angles_raw = list(data.data)
    #print(joints_angles_raw)
    '''joints_angles_rad[0] = (joints_angles_rad[0]) * POS2RAD
    joints_angles_rad[1] = (joints_angles_rad[1]) * POS2RAD
    joints_angles_rad[2] = (joints_angles_rad[2]) * POS2RAD
    joints_angles_rad[3] = joints_angles_rad[3] * DEG2RAD
    joints_angles_rad[4] = joints_angles_rad[4] * DEG2RAD'''

    joints_angles_rad[0] = (data.data[0] - ZERO_OFFSET) * POS2RAD   # <TODO> modulo or subtraction ?
    joints_angles_rad[1] = (data.data[1] - ZERO_OFFSET) * POS2RAD
    joints_angles_rad[2] = (data.data[2] - ZERO_OFFSET) * POS2RAD
    joints_angles_rad[3] = data.data[3] * DEG2RAD
    joints_angles_rad[4] = data.data[4] * DEG2RAD

    #print(joints_angles_rad)
    #location.data = data.data # joints_angles_rad
    #rospy.loginfo('location: %f %f %f', location.data[0], location.data[1], location.data[2])
    #pub_location.publish(location)
    # rospy.loginfo('is radians #################################? : %f %f', alpha, beta)


def scan_node():

    rospy.init_node('scan_node', anonymous=True)
    pub1 = rospy.Publisher("write/angles", Float32MultiArray, queue_size=10)

    rospy.Subscriber('read/angles_read', Float32MultiArray, joints_angle_read)

    a = 0
    while a <= 2:
        print('waiting {}'.format(a))
        print(joints_angles_rad)
        print(temp)
        time.sleep(1)
        a += 1
    # input('Start scan ?:')
    # rospy.Subscriber('load', Float32MultiArray, beta) # for tourqe calc

    # Creates the scan matrix
    #angles = np.array([45, 0, 0, 0, 0]) * DEG2RAD  # Insert in degree
    #world_angles_mat = calc_scan_points(angles)
    #print(joints_angles_rad)
    #rospy.loginfo(joints_angles_rad)
    #world_angles_mat = calc_scan_points(joints_angles_rad)

    current_angles = [0, 0, 0]
    j = 0
    rate = rospy.Rate(20)  # 10hz
    while not rospy.is_shutdown():
        if j == 0:
            rospy.loginfo(joints_angles_rad)
            world_angles_mat = calc_scan_points(joints_angles_rad)

        if j < len(world_angles_mat[0]):  #print(angles_mat[:, j])
            current_angles[0] = world_angles_mat[0][j] * RAD2POS + ZERO_OFFSET
            # current_angles[0] = current_angles[0] % 4096
            current_angles[1] = world_angles_mat[1][j] * RAD2POS + ZERO_OFFSET
            current_angles[2] = world_angles_mat[2][j] * RAD2POS + ZERO_OFFSET
            angles_msg.data = current_angles
            j += 1
            rospy.loginfo(angles_msg)
            pub1.publish(angles_msg)
            rate.sleep()  # '''

    # rospy.spin()


def calc_scan_points(angles):

    [a04, q4, q5] = sensor_location(angles[0], angles[1], angles[2], angles[3], angles[4])
    d = 2
    plain_offset = -l4
    x7 = np.array(range(-25, 27, d))
    y7 = np.array(range(-25, 27, d))
    #   xx7, yy7 = np.meshgrid(x7, y7)
    z7 = plain_offset * np.ones(len(y7))
    size = len(y7) ** 2
    scan_matrix = list(np.array([np.ones(size), np.ones(size), np.ones(size), np.ones(size)]))
    #  scan_matrix[-1][-1] = 1 # unneccesery
    i = 0
    j = 0
    index = 0

    while index < size:

        scan_matrix[0][index] = x7[i]
        scan_matrix[1][index] = y7[j]
        scan_matrix[2][index] = z7[i]
        i += 1
        index += 1

        if x7[i] == -25 or x7[i] == 25:
            scan_matrix[0][index] = x7[i]
            scan_matrix[1][index] = y7[j]
            scan_matrix[2][index] = z7[i]
            x7 = np.flipud(x7)
            j += 1
            index += 1
            i = 0  # ??????

    world_point_mat = a04.dot(scan_matrix)
    # print(world_point_mat)

    # calc World's points
    h = 0
    world_angles = (np.array([np.zeros(size), np.zeros(size), np.zeros(size)]))
    while h < len(world_point_mat[0]):
        some_angles = joints_angles(world_point_mat[0][h], world_point_mat[1][h], world_point_mat[2][h])
        world_angles[0][h] = some_angles[0]
        world_angles[1][h] = some_angles[1]
        world_angles[2][h] = some_angles[2]
        h += 1

    #  world_angles = np.transpose(world_angles)
    return world_angles


def sensor_location(t1, t2, t3, alpha, beta):

    l0 = 40
    l1 = 44
    l2 = 108
    l3 = 60
    l4 = 31.5

# Base rotation about z axis
    c1 = math.cos(t1)
    s1 = math.sin(t1)
    a01 = np.array(((c1, -s1, 0, 0), (s1, c1, 0, 0), (0, 0, 1, l0), (0, 0, 0, 1)))
    # print(a01)

# 2nd joint rotation about x axis
    c2 = math.cos(t2)
    s2 = math.sin(t2)
    a12 = np.array(((1, 0, 0, 0), (0, c2, -s2, 0), (0, s2, c2, l1), (0, 0, 0, 1)))
    # print(a12)

# 3rd joint rotation about x axis
    c3 = math.cos(t3)
    s3 = math.sin(t3)
    a23 = np.array(((1, 0, 0, 0), (0, c3, -s3, 0), (0, s3, c3, l2), (0, 0, 0, 1)))
    # print(a23)

# Displacement in Z3 coordinate system
    a34 = np.array(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, l3), (0, 0, 0, 1)))

    ca = math.cos(alpha)
    sa = math.sin(alpha)
    ax = np.array(((1, 0, 0, 0), (0, ca, -sa, 0), (0, sa, ca, 0), (0, 0, 0, 1)))

    cb = math.cos(beta)
    sb = math.sin(beta)
    ay = np.array(((cb, 0, sb, 0), (0, 1, 0, 0), (-sb, 0, cb, 0), (0, 0, 0, 1)))
    am = np.array(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, l4), (0, 0, 0, 1)))

    c4 = math.cos(-t1)
    s4 = math.sin(-t1)
    a_offset_rotation = np.array(((c1, -s1, 0, 0), (s1, c1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))

    a04 = a01.dot(a12.dot(a23.dot(a34.dot(ax.dot(ay.dot(am.dot(a_offset_rotation)))))))
    # print(a04)
    p0 = a04.dot([0, 0, 0, 1])
    # print(p0)
    q3 = a01.dot(a12.dot(a23.dot([0, 0, 0, 1])))
    q4 = a01.dot(a12.dot(a23.dot(a34.dot([0, 0, 0, 1]))))
    # q5 = a01.dot(a12.dot(a23.dot(a34.dot([0, 0, l4, 1]))))
    q5 = a01.dot(a12.dot(a23.dot(a34.dot(ax.dot(ay.dot(am.dot([0, 0, 0, 1])))))))
    # origin = [0, 0, 0]
    first_link = [0, 0, l0]
    second_link = [0, 0, l1+l0]
    third_link = [q3[0], q3[1], q3[2]]
    forth_link = [q4[0], q4[1], q4[2]]
    fifth_link = [q5[0], q5[1], q5[2]]

    return a04, q4, q5


if __name__ == '__main__':
    try:
        scan_node()
    except rospy.ROSInterruptException:
        pass
