from modern_robotics import se3ToVec, MatrixLog6, TransInv, FKinBody, JacobianBody
import numpy as np
import csv


def IKinBodyIterates(Blist,M,T,thetalist0,eomg,ev):
#Takes Blist: The joint screw axes in the end-effector frame when the
#             manipulator is at the home position,
#      M: The home configuration of the end-effector,
#      T: The desired end-effector configuration Tsd,
#      thetalist0: An initial guess of joint angles that are close to
#                  satisfying Tsd,
#      eomg: A small positive tolerance on the end-effector orientation
#            error. The returned joint angles must give an end-effector
#            orientation error less than eomg,
#      ev: A small positive tolerance on the end-effector linear position
#          error. The returned joint angles must give an end-effector
#          position error less than ev.
#Returns thetalist: Joint angles that achieve T within the specified
#                   tolerances,
#        success: A logical value where TRUE means that the function found
#                 a solution and FALSE means that it ran through the set
#                 number of maximum iterations without finding a solution
#                 within the tolerances eomg and ev.
#Uses an iterative Newton-Raphson root-finding method.
#The maximum number of iterations before the algorithm is terminated has
#been hardcoded in as a variable called maxiterations. It is set to 20 at
#the start of the function, but can be changed if needed.
#Prints a report of joint vector, end-effector config, and angular and linear errors for each iteration
#Saves joint vectors for all iterations as a csv file
    '''
Example Input:
import numpy as np
Blist = np.array([[0, 0, -1, 2, 0,   0],
                  [0, 0,  0, 0, 1,   0],
                  [0, 0,  1, 0, 0, 0.1]]).T
M = [[-1, 0, 0, 0], [0, 1, 0, 6], [0, 0, -1, 2], [0, 0, 0, 1]]
T = [[0, 1, 0, -5], [1, 0, 0, 4], [0, 0, -1, 1.6858], [0, 0, 0, 1]]
thetalist0 = [1.5, 2.5, 3]
eomg = 0.01
ev = 0.001
Output:
thetalist:
[1.57073819, 2.999667, 3.14153913]
success:
True
    '''
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M,Blist, \
                                                 thetalist)),T)))
    angerror = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
    linerror = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
    err = angerror > eomg or linerror > ev

    # print report for initial values
    print('Iteration 0:')
    print(f'joint vector: {thetalist}')
    print(f'SE(3) end-effector config: {Vb}')
    print(f'angular error magnitude ||omega_b||: {angerror}')
    print(f'linear error magnitude ||v_b||: {linerror}\n')

    # save the initial guess of joint vectors
    iterations = [thetalist]

    while err and i < maxiterations:
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(JacobianBody(Blist, \
                                                         thetalist)),Vb)
        i = i + 1
        Vb \
        = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M,Blist, \
                                                       thetalist)),T)))
        angerror = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        linerror = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
        err = angerror > eomg or linerror > ev

        # add the new row of joint vectors
        iterations.append(thetalist)

        # print report for current iteration
        print(f'Iteration {i+1}:')
        print(f'joint vector: {thetalist}')
        print(f'SE(3) end-effector config: {Vb}')
        print(f'angular error magnitude ||omega_b||: {angerror}')
        print(f'linear error magnitude ||v_b||: {linerror}\n')

    # save csv file
    with open('iterates.csv', 'w') as csvfile:
        iterateswriter = csv.writer(csvfile)
        iterateswriter.writerows(iterations)

    return (thetalist,not err)