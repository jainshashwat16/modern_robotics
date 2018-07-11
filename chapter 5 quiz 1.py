import numpy as np
import math as m
from modern_robotics import JacobianSpace
from modern_robotics import JacobianBody
from modern_robotics import Normalize
#question 1
J = [[0, 0, 0], [0, 0, 0], [1, 1, 1], [0, 0, .707], [0, -1, -1.707], [0, 0, 0]]
Jt = np.array(J).transpose()
F_s = np.array([0, 0, 0, 2, 0, 0]).transpose()
F_b = np.matmul(Jt, F_s)
print('Question 1:')
print(F_b)

#question 2
pi = 3.14159
a1 = 0
a2 = 0
a3 = pi/2
a4 = -pi/2


J_b = [[1, 1, 1, 1], [m.sin(a4)+m.sin(a3+a4)+m.sin(a2+a3+a4), m.sin(a4)+m.sin(a3+a4), m.sin(a4), 0], [1+m.cos(a4)+m.cos(a3+a4)+m.cos(a2+a3+a4), 1+m.cos(a4)+m.cos(a3+a4), 1+m.cos(a4), 1]]
F_b = [10, 10, 10]
t_b = np.matmul(np.array(J_b).transpose(), F_b)
print('Question 2')
print(t_b)

#question 3
S_list = np.array([[0,0,1,0,0,0],[1,0,0,0,2,0],[0,0,0,0,1,0]]).T
J_s = JacobianSpace(S_list, [pi/2,pi/2,1])
print('Question 3')
print(J_s)

#question 4
B_list = np.array([[0,1,0,3,0,0],[-1,0,0,0,3,0],[0,0,0,0,0,1]]).T
J_b = JacobianBody(B_list,[pi/2,pi/2,1])
print('Question 4')
print(J_b)

#question 5
J_v = np.array([[-.105,0,.006,-.045,0, .006,0], [-.889,.006,0,-.844,.006,0,0], [0,-.105,.889,0,0,0,0]])
e_vals, e_vecs = np.linalg.eig(np.matmul(J_v,np.array(J_v).transpose()))
print("question 5")
print(Normalize(e_vecs[1]))

print("Question 6")
print(m.sqrt(e_vals.max()))
