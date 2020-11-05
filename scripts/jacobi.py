import math as m
import numpy as np


def jacobi(thetas):

    t = np.array(thetas)
    l2 = 108
    l3 = 60

    ''' this func. takes INPUT: joints' angles 
                    and gives OUTPUT: the Jacobian matrix    
        the calculations based on the inverse kinematics, three angles for three joints respectively '''

    dxdt1 = m.cos(t[0]) * (l2 * m.sin(t[1]) + l3 * m.sin(t[1] + t[2]))
    dxdt2 = m.sin(t[0]) * (l2 * m.sin(t[1]) + l3 * m.sin(t[1] + t[2]))
    dxdt3 = l3 * m.sin(t[0]) * m.cos(t[1] + t[2])

    dydt1 = m.sin(t[0]) * (l2 * m.sin(t[1]) + l3 * m.sin(t[1] + t[2]))
    dydt2 = -m.cos(t[0]) * (l2 * m.sin(t[1]) + l3 * m.sin(t[1] + t[2]))
    dydt3 = -l3 * m.cos(t[0]) * m.cos(t[1] + t[2])

    dzdt1 = 0
    dzdt2 = -l2 * m.sin(t[1] + t[2])
    dzdt3 = -l3 * m.sin(t[1] + t[2])

    j_mat = np.array([[dxdt1, dxdt2, dxdt3], [dydt1, dydt2, dydt3], [dzdt1, dzdt2, dzdt3]])
    det_j = np.linalg.det(j_mat)

    '''if det_j == 0:
        print('singular')
        return False'''

    return j_mat


def force_calc(j_mat, torque, thetas):

    j_matT = np.transpose(j_mat)
    j_matTI = np.linalg.inv(j_matT)

    ''' this func. takes INPUT: joints' angles, the matches Jacobyan matrix, and the joints' angles    
                        and gives OUTPUT: the force act on the arm's tip '''

    '''  # this is only for debugging i.e delete later
    print(j_mat)
    print('')
    print(j_matT)
    print('')
    print(j_matTI)'''

    # tourqe = np.array([-1, -1, 0]) * 0.001  # The unit has to be N*mm
    # f = -j_matTI @ torque

    f = -j_matTI.dot(torque)
    a04, q4, q5 = sensor_location(thetas[0], thetas[1], thetas[2], thetas[3], thetas[4])
    unitvec = (q5-q4) / 31.5
    unitvec = unitvec[:-1]
    norm_force = abs(f.dot(unitvec))
    ff = np.append(f, norm_force)

    return ff


def sensor_location(t1, t2, t3, alpha, beta):

    l0 = 40
    l1 = 44
    l2 = 108
    l3 = 60
    l4 = 31.5

# Base rotation about z axis
    c1 = m.cos(t1)
    s1 = m.sin(t1)
    a01 = np.array(((c1, -s1, 0, 0), (s1, c1, 0, 0), (0, 0, 1, l0), (0, 0, 0, 1)))
    # print(a01)

# 2nd joint rotation about x axis
    c2 = m.cos(t2)
    s2 = m.sin(t2)
    a12 = np.array(((1, 0, 0, 0), (0, c2, -s2, 0), (0, s2, c2, l1), (0, 0, 0, 1)))
    # print(a12)

# 3rd joint rotation about x axis
    c3 = m.cos(t3)
    s3 = m.sin(t3)
    a23 = np.array(((1, 0, 0, 0), (0, c3, -s3, 0), (0, s3, c3, l2), (0, 0, 0, 1)))
    # print(a23)

# Displacement in Z3 coordinate system
    a34 = np.array(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, l3), (0, 0, 0, 1)))

    ca = m.cos(alpha)
    sa = m.sin(alpha)
    ax = np.array(((1, 0, 0, 0), (0, ca, -sa, 0), (0, sa, ca, 0), (0, 0, 0, 1)))

    cb = m.cos(beta)
    sb = m.sin(beta)
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


# j_mat = jacobi([0, m.pi/6, m.pi/6])
# force_calc(j_mat)