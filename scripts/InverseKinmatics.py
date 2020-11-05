import math
import numpy as np
import matplotlib.pyplot as plt

''' this func. takes INPUT: location 
            and give OUTPUT: joints' angles    
the calculations based on the inverse kinematics, three angles for three joints respectively '''


def joints_angles(x, y, z):
    # the func gets point and return the angles of the joints

    l0 = 40
    l1 = 44
    l2 = 108
    l3 = 60
    l4 = 31.5
    z2 = z - (l0 + l1)
    d = math.sqrt(z2 ** 2 + y ** 2 + x ** 2)

    # Need to a add cases for x = 0**
    # teta1 = math.atan(y / x)
    teta1 = math.atan2(y, x) + math.pi/2
    arg = ((d ** 2 - (l3 ** 2 + l2 ** 2)) / (2 * l2 * l3))
    '''if arg > 1:
        teta3 = math.acos(0.99)
    elif arg < -1:
        teta3 = math.acos(-0.99)
    else:'''
    teta3 = math.acos((d ** 2 - (l3 ** 2 + l2 ** 2)) / (2 * l2 * l3))
    teta2 = math.atan2(math.sqrt(y ** 2 + x ** 2), z2) - math.asin((l3 * math.sin(teta3)) / d)

    teta1_deg = teta1 * 180 / math.pi
    teta2_deg = teta2 * 180 / math.pi
    teta3_deg = teta3 * 180 / math.pi
    '''print(teta1_deg)
    print(teta2_deg)
    print(teta3_deg)'''

    """teta1 = math.pi/10 + math.pi/2
    teta2 = math.pi/10
    teta3 = math.pi/10"""

    '''teta1 = 104.03/180.0*math.pi + math.pi/2.0
    teta2 = (89.56)/180.0*math.pi
    teta3 = (95.20)/180.0*math.pi'''

    return teta1, teta2, teta3


# joints_angles(-25, 40, -25)
