from modern_robotics import FKinBody, TransInv, FKinSpace
import numpy as np
import math
from decimal import Decimal


def calc_t_se(configuration, M, b_list, t_b0):
    t_0e = FKinBody(M, b_list, configuration[3:8])
    t_sb = calc_t_sb(configuration[0], configuration[1], configuration[2], .0963)
    t_s0 = np.dot(t_sb, t_b0)
    t_se = np.dot(t_s0, t_0e)
    return t_se


def to_decimal(array):
    for row in array:
        if isinstance(row, list):
            for entry in row:
                entry = Decimal(entry)
        else:
            row = Decimal(row)
    return array


def to_float(array):
    for row in array:
        if isinstance(row, list):
            for entry in row:
                entry = float(entry)
        else:
            row = float(row)
    return array


def calc_t_sb(phi, x, y, z):
    """Returns at transformation matrix for the base
    Inputs
    phi (float): base angle of rotation in space frame
    x (float): base x coordinate in space frame
    y (float): base y coordinate in space frame
    z (float): constant base height
    Output
    t (array in SE3): base transformation matrix in the space frame
    """
    t = [[math.cos(phi), -math.sin(phi), 0, x],
         [math.sin(phi), math.cos(phi), 0, y],
         [0, 0, 1, z],
         [0, 0, 0, 1]]
    return t


def calc_f_matrix():
    """Returns the F matrix for the base
    F (array of floats): the chassis F matrix
    """
    l = Decimal(.235)
    w = Decimal(.150)
    wheel_rad = Decimal(.0475)
    F = np.array([  [Decimal(-1)/(l + w), Decimal(1)/(l+w), Decimal(1)/(l+w), Decimal(-1)/(l+w)],
                    [Decimal(1), Decimal(1), Decimal(1), Decimal(1)],
                    [Decimal(-1), Decimal(1), Decimal(-1), Decimal(1)]])
    F = wheel_rad * Decimal(.25) * F
    F = np.array([[float(i[0]), float(i[1]), float(i[2]), float(i[3])] for i in F])
    return F