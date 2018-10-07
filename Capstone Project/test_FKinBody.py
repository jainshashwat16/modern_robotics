from modern_robotics import FKinBody
import numpy as np
M = np.array([[-1, 0, 0, 0], [0, 1, 0, 6], [0, 0, -1, 2],
              [0, 0, 0, 1]])
Blist = np.array([[0, 0, -1, 2, 0, 0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 1, 0, 0, 0.1]]).T
thetalist = np.array([np.pi / 2.0, 3, np.pi])

t = FKinBody(M, Blist, thetalist)

print(t)