import math
import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

''' this func. takes INPUT: angles 
            and give OUTPUT: location
the calculations based on direct kinematics, each matrix represent a 3D space transformation;
                                                                rotational or displacement. '''


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
    a04 = a01.dot(a12.dot(a23.dot(a34.dot(ax.dot(ay.dot(am))))))
    _a04_ = a01.dot(a12.dot(a23.dot(a34.dot(ax.dot(ay.dot(am))))))
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
    # print(q4)
    #return a04, q4, q5
    #a = np.array((first_link, second_link, third_link, forth_link, fifth_link))
    return a04, q4, q5

# sensor_location(math.pi/180*30, math.pi/180*60, math.pi/180*30, math.pi/180*0, math.pi/180*0)
