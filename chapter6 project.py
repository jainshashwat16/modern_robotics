from IKinBodyIterates import IKinBodyIterates
import numpy as np

# example code demonstrating usage of the IKinBodyIterates function

e_w = .001
e_v = .0001
W1 = .109
W2 = .082
L1 = .425
L2 = .392
H1 = .089
H2 = .095

M = [[-1,0,0,L1+L2],[0,0,1,W1+W2],[0,1,0,H1-H2],[0,0,0,1]]
B = np.transpose(np.array([[0,1,0,W1+W2,0,L1+L2],[0,0,1,H2,-L1-L2,0],[0,0,1,H2,-L2,0],[0,0,1,H2,0,0],[0,-1,0,-W2,0,0],[0,0,1,0,0,0]]))
theta_0 = (2.14,-5.52,3.93,-2.27,.69,-4.969)
T_sd = [[0,1,0,-.5],[0,0,-1,.1],[-1,0,0,.1],[0,0,0,1]]

IKinBodyIterates(B, M, T_sd, theta_0, e_w, e_v)
