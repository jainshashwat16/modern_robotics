import numpy as np
import csv
from scipy import optimize
import math

"""This code takes a .csv file containing the coordinates, and normal angle (in radians) of a set planar contacts. It
returns whether or not the set is in form closure. If closed it returns the solution to the linear problem.

Example usage log:


Input filename including the extension for contact set:
closed.csv
Form is closed
[3.         1.         4.24264069 2.        ]

Where closed.csv contains the following contact data (x coordinate, y coordinate, normal direction in radians):
0	1	0
0	3	4.71238898
2	1	2.35619449
3	3	4.71238898
"""

# get the filename from the user
file_name = input("Input filename including the extension for contact set:\n")


# import contact set
contacts = []
with open(file_name) as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
    for row in reader:
        """iterate through row. Calculate (x,y) components to unit vector and the cross product of the contact normal 
        its displacement vector"""
        x = math.cos(row[2])
        y = math.sin(row[2])
        m = np.cross([x, y], [row[0], row[1]])

        contacts.append([x, y, m])
contacts = np.array(contacts).T

# check rank
if np.linalg.matrix_rank(contacts) == 3:

    # evaulate form closure using the linpro function from the scipy library
    c = [1] * contacts.shape[1]  # Coefficients of the linear objective function to be minimized
    A_ub = np.identity(contacts.shape[1]) * -1  # Array that when multiplied by X gives the values of the upper bound
    A_eq = contacts  # array that when multiplied by x gives the values of the equality constraints at x
    b_ub = [-1] * contacts.shape[1] # array of values representing the upper bound of each row in A_ub
    b_eq = [0, 0, 0] # RHS of each equality constraint in A_eq
    result = optimize.linprog(c, A_ub, b_ub, A_eq, b_eq, options={'tol': .000001})
    if result.success == True:
        print("Form is closed")
        print(result.x)
    else:
        print("Form is open")
else:
    print("Form is open")

