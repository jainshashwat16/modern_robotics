import numpy as np
import math
from scipy import optimize

"""This program determines if an assembly of bodies in frictional contact will remain standing. If the assembly stands
the program will return the magnitude of the contact force on each edge of each friction cone.

To use two files are needed. 
Firstly, a body file containing the information in about each body with the following columns:
mass, x coordinate of cg, y coordinate of cg
Secondly a contacts file containing information about the contacts between each body should be included. It should 
have the following columns:
contact x coordinate, contact y coordinate, coefficient of friction, first body number, second body number, contact normal
direction relative to the first body in radians.
Note, the ground should be assigned the body number "0"

Example usage: 

Input filename including the extension for the body data file:
standing assembly bodies.csv
Input filename including the extension for the contact data file:
standing assembly contacts.csv
Assembly stands
k =[ 3.351 15.538  0.     6.093  0.    27.42  62.456 22.85 ]

"""


# query user for input files
body_file = input("Input filename including the extension for the body data file:\n")
contact_file = input("Input filename including the extension for the contact data file:\n")

# import the bodies and contacts data
bodies = np.genfromtxt(body_file, delimiter=",", skip_header=1)
contacts = np.genfromtxt(contact_file, delimiter=",", skip_header=1)

# create a matrix of unit forces and their corresponding moments
coefficients = np.zeros((3 * bodies.shape[0], contacts.shape[0] * 2 + bodies.shape[0]), dtype=float)

for body_no, body in enumerate(bodies):
    # select all contacts where one of the bodies in the contact is the selected body
    for contact_no, contact in enumerate(contacts):
        if contact[3] == body_no + 1 or contact[4] == body_no + 1:
            # calculate the angle of each edge of the friction cone
            alpha = math.atan(contact[2])
            theta_1 = contact[5] + alpha
            theta_2 = contact[5] - alpha

            # reverse the direction of force for the second body in the contact set except where the first body is ground
            if contact[4] == body_no + 1 and contact[3] != 0:
                direction = -1
            else:
                direction = 1

            # calculate the x and y components of the unit vector representing each edge of the friction cone
            # calculate the magnitude of the moment created by the unit force. All moments are about the origin
            N_x1 = math.cos(theta_1) * direction
            N_y1 = math.sin(theta_1) * direction
            m1 = np.cross([contact[0], contact[1]], [N_x1, N_y1])
            N_x2 = math.cos(theta_2) * direction
            N_y2 = math.sin(theta_2) * direction
            m2 = np.cross([contact[0], contact[1]], [N_x2, N_y2])

            # add the wrenches for each edge of the friction cone to the coefficients matrix in the correct position
            new_wrenches = np.array([[N_x1, N_y1, float(m1)], [N_x2, N_y2, float(m2)]]).T
            coefficients[body_no*3:body_no*3 + new_wrenches.shape[0], contact_no * 2:contact_no * 2 + new_wrenches.shape[1]] = new_wrenches

    # calculate gravitational wrench on the body and add it to the coefficients matrix
    fg_y = -body[0] * 9.81
    mg_y = np.cross([body[1], body[2]], [0, fg_y])
    gravitational_wrench = np.array([0, fg_y, mg_y])
    coefficients[body_no * 3:body_no * 3 + gravitational_wrench.shape[0], contacts.shape[0] * 2 + body_no] = gravitational_wrench.T

# evaluate force closure using the linpro function from the scipy library
c = [1] * coefficients.shape[1]  # Coefficients of the linear objective function to be minimized

# each row should sum to zero
A_eq = coefficients  # array that when multiplied by x gives the values of the equality constraints at x
b_eq = np.zeros(coefficients.shape[0])  # RHS of each equality constraint in A_eq

# the coefficients for the gravity terms should be equal to 1, coefficients for all other terms should be >=0
upper_bound = [None] * contacts.shape[0] * 2
lower_bound = [0] * contacts.shape[0] * 2
upper_bound += [1] * bodies.shape[0]
lower_bound += [1] * bodies.shape[0]

result = optimize.linprog(c, A_eq=A_eq, b_eq=b_eq, bounds=list(zip(lower_bound, upper_bound)), options={'tol': .000001})
if result.success:
    print('Assembly stands')
    np.set_printoptions(precision=3, suppress=True)
    print("k =" + str(result.x[:-2]))
else:
    print("Assembly collapses")
