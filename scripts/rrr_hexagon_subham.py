#!/usr/bin/env python
from math import *
import rospy
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from time import sleep

def inv_kin(px, py, fi):
    l1=1
    l2=1
    l3=1

    wx = px - l3*cos(fi)
    wy = py - l3*sin(fi)
    D = (wx**2 + wy**2 - l1**2 -l2**2)/(2*l1*l2)

    theta2 = atan2(sqrt(1 - D**2), D)
    theta1 = atan2(wy, wx) - atan2((l2*sin(theta2)), (l1 + l2*cos(theta2)))
    theta3 = fi - theta1 - theta2
    return theta1, theta2, theta3



def myltiply_two_mat(A, B):
    result = [[0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]]
  
    # iterating by row of A
    for i in range(len(A)):
    
        # iterating by coloum by B 
        for j in range(len(B[0])):
    
            # iterating by rows of B
            for k in range(len(B)):
                result[i][j] += A[i][k] * B[k][j]
    return result



def origin_of_end_effctor(On_0, Rotation_mat, den):
    result = [0, 0, 0]
    p_e_o = [0, 0, 0]
    # iterating by row of A
    for i in range(len(Rotation_mat)):
    
        # iterating by coloum by B 
        for j in range(len(den)):
            result[i] += Rotation_mat[i][j] * den[j]
    # return result
    for i in range(len(result)):
        p_e_o[i] = On_0[i] +result[i]

    return p_e_o



def for_kin_rrr(theta, XX, YY):

    # sample input
    # theta1 = pi/6
    # theta2 = pi/6
    # theta3 = pi/6
    # theta = [theta1, theta2, theta3, pi]

    n =3
    l1 =1
    l2 =1
    l3 =1
    # DH Parameters
    alpha = [0, 0, 0]
    ai = [0, l1, l2]
    di = [0, 0, 0]

    den = [l3, 0, 0]
    Temp = [[1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]

    Ti0 = []
    Oi0 = []
    # XX=[]
    # YY=[]
    for i in range(n):
        # Transformation matrix of frame i with respect to i-1 frame
        T_i_i_1 = [ [cos(theta[i]),                        -sin(theta[i]),                  0,                        ai[i] ],
                    [sin(theta[i])*cos(alpha[i]),         cos(theta[i])*cos(alpha[i]),     -sin(alpha[i]),            -di[i]*sin(alpha[i])],
                    [sin(theta[i])*sin(alpha[i]),          cos(theta[i])*sin(alpha[i]),     cos(alpha[i]),            di[i]*cos(alpha[i])],
                    [0,                                      0,                                  0,                            1]]

        # Transformation matrix of frame i with respect to 0th frame
        Ti0.append( myltiply_two_mat(Temp, T_i_i_1))
        Temp=Ti0[::][::][i]

        # Origin of ith link with respect to 0th link
        Oi0.append([row[3] for row in Temp[0:3]])

        # X = [row[0] for row in Oi0]
    Rotation_mat_n_0 = [row[:3] for row in Temp[0:3]]
    
    Pe0 = origin_of_end_effctor(Oi0[n-1], Rotation_mat_n_0, den)
    Oi0.append(Pe0)
    X = [row[0] for row in Oi0]
    Y = [row[1] for row in Oi0]

    XX.append(X[-1])
    YY.append(Y[-1])
    # print(Oi0)
    # print(X)
    plt.figure(100)
    plt.plot(X,Y)
    plt.plot(XX,YY,'-')
    plt.xlim(-3,3.1)
    plt.ylim(-3,3.1)
    plt.xlabel("X Axis")
    plt.ylabel("Y Axis")
    plt.title("RRR Planer Manipulator")
    plt.pause(0.1)
    plt.clf()


def rrr_publisher(theta1, theta2, theta3):
    pub1 = rospy.Publisher('/rrr/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/rrr/joint2_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/rrr/joint3_position_controller/command', Float64, queue_size=10)




    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    msg1 = Float64()
    msg2 = Float64()
    msg3 = Float64()
    # while not rospy.is_shutdown():

    msg1.data = theta1
    msg2.data = theta2
    msg3.data = theta3


    pub1.publish(msg1)
    pub2.publish(msg2)
    pub3.publish(msg3)
    rate.sleep()




def setInitialPose(px,py,fi):
    theta1, theta2, theta3 = inv_kin(px, py, fi)
    rrr_publisher(theta1, theta2, theta3)
    sleep(10)



def main():
    a = 1.7
    b=0.7
    p_x = [0+a, 0.65+a, 0.65+a, 0+a, -0.65+a, -0.65+a, 0+a]
    p_y = [1+b, 0.6+b, -0.13+b, -0.5+b, -0.2+b, 0.6+b, 1+b]

    # p_x = [1, 1.65, 1.65, 1, 0.35, 0.35, 1]
    # p_z = [4+1, 3.6+1, 2.87+1, 2.5+1, 2.87+1, 3.6+1, 4+1]
    fi=0
    P_start_x = p_x[0]  
    P_start_y = p_y[0]


    setInitialPose(P_start_x,P_start_y,fi)
    del_t = 0.01
    t=0
    i=0
    time = []
    theta1_stacked = []
    theta2_stacked = []
    theta3_stacked = []
    XX=[]
    YY=[]
    while True:
        x_dot = p_x[i+1] - p_x[i]
        y_dot = p_y[i+1] - p_y[i]

        p_next_x = P_start_x + del_t* x_dot
        p_next_y = P_start_y + del_t* y_dot
        
        theta1, theta2, theta3 = inv_kin(p_next_x, p_next_y, fi)
        
        error = sqrt((p_next_x- p_x[i+1])**2 + (p_next_y - p_y[i+1])**2)

        if error <= 0.01:
            if i<5:
                i+=1
            else :
                break

        rrr_publisher(theta1, theta2, theta3)
        for_kin_rrr([theta1, theta2, theta3],XX,YY)

        # plt.figure(2)
        # plt.plot(p_next_x, p_next_z,"ob")
        # # plt.xlim(-3,3)xacro: in-order processing became default in ROS Melodic. You can drop the option.

        # # plt.ylim(-3,3)
        # plt.xlabel("X Axis")
        # plt.ylabel("Y Axis")
        # plt.title("End Effector Motion")
        # plt.pause(0.1)
        theta1_stacked.append(theta1)
        theta2_stacked.append(theta2)
        theta3_stacked.append(theta3)
        # time.append(t)
        # t = t+del_t
        P_start_x = p_next_x
        P_start_y = p_next_y
     
    # plt.figure(2)  
    # plt.plot(time,theta1_stacked)
    

    # plt.figure(3)  setInitialPose()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass