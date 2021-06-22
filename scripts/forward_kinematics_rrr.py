from math import *
import matplotlib.pyplot as plt
import numpy as np



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



if __name__ == "__main__":
    xx = 0
    theta1=0
    theta2=0
    theta3=0
    XX=[]
    YY=[]
    for i in range(100):
        # theta = [pi/6, pi/6, pi/6, pi/2]
        theta = [theta1+xx, theta2, theta3]
        for_kin_rrr(theta, XX, YY)
        xx = xx+0.1