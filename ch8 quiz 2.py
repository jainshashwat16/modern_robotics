import numpy as np
from modern_robotics import MassMatrix
from modern_robotics import VelQuadraticForces
from modern_robotics import GravityForces
from modern_robotics import EndEffectorForces
from modern_robotics import ForwardDynamics

# Question 1
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67]
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]

Slist = np.array(Slist)

theta = [0, np.pi/6, np.pi/4, np.pi/3, np.pi/2, 2*np.pi/3]
print(theta)
thetadot = [.2, .2, .2, .2, .2, .2]
thetadotdot = [.1, .1, .1, .1, .1, .1]
g = [0, 0, -9.81]
ftip = [.1, .1, .1, .1, .1, .1]

theta =np.array(theta)
thetadot = np.array(thetadot)
thetadotdot = np.array(thetadot)
g = np.array(g)
ftip = np.array(ftip)

print("Question 1")
print(MassMatrix(theta, Mlist, Glist, Slist))

#question 2
print("Question 2")
print(VelQuadraticForces(theta, thetadot, Mlist, Glist, Slist))

#question 3
print("Question 3")
print(GravityForces(theta, g, Mlist, Glist, Slist))

#question 4
print("Question 4")
print(EndEffectorForces(theta, ftip, Mlist, Glist, Slist))

#question 5
print("Question 5")
tau = [.0128, -41.1477, -3.7809, .0323, .0370, .1034]
print(ForwardDynamics(theta, thetadot, tau, g, ftip, np.array(Mlist), Glist, Slist))