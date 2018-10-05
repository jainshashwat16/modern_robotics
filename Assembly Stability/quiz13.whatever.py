from modern_robotics import MatrixExp3
from modern_robotics import so3ToVec
from modern_robotics import Adjoint
import numpy as np

#question 5
Vb = np.array([[0, -.33, -.25], [.33, 0, -.12], [.25, .12, 0]])

print(so3ToVec(MatrixExp3(Vb)))

#question 6
Teb = [[0,-1,0,2], [1,0,0,3],[0,0,1,0],[0,0,0,1]]
r = .5
d = 1
F = [[-r/(2*d), r/(2*d)], [r/2, r/2]]
print(Adjoint(Teb))
