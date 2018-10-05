import numpy as np
import math

# wheel 1 calcs


def row_calc(x, y, b):
    z = np.array([1/.25, 0])
    h1 = np.array([[math.cos(b), math.sin(b)], [-math.sin(b), math.cos(b)]])
    h2 = np.array([[-y, 1, 0], [x, 0, 1]])
    row = np.matmul(h1, h2)
    row = np.matmul(z, row)
    return row

# question 1
H = [row_calc(2, 2, -math.pi/4), row_calc(-2, 2, math.pi/4), row_calc(-2, -2, 3*math.pi/4), row_calc(2, -2, -3*math.pi/4)]
print(H)
v = [1, 0, 0]
u = np.matmul(H, v)
print(u)

# question 2
v = [1, 2, 3]
u = np.matmul(H, v)
print(u)