import numpy
import math
from modern_robotics import IKinBody

theta = [1, 1]
x_d = [0, 0]


def f(theta):
    f_x = math.pow(theta[0], 2) - 9
    f_y = math.pow(theta[1], 2) - 4
    return numpy.transpose(numpy.array([f_x, f_y]))


def j(theta):
    fprime_x = 2 * theta[0]
    fprime_y = 2 * theta[1]
    return numpy.array([[fprime_x, 0], [0, fprime_y]])


print('Question 1:')

for i in range(0, 2):
    print('iteration')
    e = numpy.transpose(numpy.array(x_d)) - f(theta)
    print(e)
    theta = numpy.transpose(numpy.array(theta))
    print(theta)
    theta = theta + numpy.matmul(numpy.linalg.inv(j(theta)), e)
    print(theta)

print('Question 2:')
e_w = .001
e_v = .0001
L_1 = .550
L_2 = .3
L_3 = .06

M = [[1,0,0,0],[0,1,0,0],[0,0,1, L_1 + L_2 + L_3], [0,0,0,1]]
B = numpy.transpose(numpy.array([[0,0,1,0,0,0],[0,1,0, L_1+L_2+L_3, 0,0], [0,0,1,0,0,0],[0,1,0,L_2+L_3,0,.045],[0,0,1,0,0,0],[0,1,0,L_3,0,0],[0,0,1,0,0,0]]))
theta_0 = (1, 1, 1, -1, 1, 1, -1)
T_sd = [[1,0,0,.5],[0,1,0,0],[0,0,1,.6],[0,0,0,1]]

print(IKinBody(B,M,T_sd,theta_0,e_w,e_v))