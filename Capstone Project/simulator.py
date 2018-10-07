import numpy as np
import math
import modern_robotics
from decimal import Decimal
from utils import to_decimal, to_float, calc_t_sb, calc_f_matrix


def next_state(configuration, joint_speeds, time_step, max_speed):
    """Calculates the next configuration of the robot given the current configuration and joint speeds
    Input
    configuration (13 vector of floats):
    joint speeds (9 vector of floats):
    time step (float): interval to use for numerical integration
    max_speed (float): speed limit for joint and wheel velocities
    """

    joint_angles, delta_theta = euler_step(configuration, joint_speeds, time_step, max_speed)
    # update chassis configuration using odometry

    F = calc_f_matrix()

    V_b = np.dot(F, delta_theta[5:9])
    print(V_b)

    T_sb_k = calc_t_sb(configuration[0], configuration[1], configuration[2], .0963)

    T_bbprime = calc_t_sb(V_b[0], V_b[1], V_b[2], 0)
    T_k_plus_1 = np.dot(T_sb_k, T_bbprime)

    q_k_plus_1 = [math.acos(T_k_plus_1[0, 0]), T_k_plus_1[0, 3], T_k_plus_1[1, 3]]
    print(q_k_plus_1)

    new_configuration = np.concatenate([q_k_plus_1, np.array(joint_angles[3:])])

    return new_configuration


def euler_step(configuration, joint_speeds, time_step, max_speed):
    """calculate the new joint and wheel angles using a first-order Euler step
    Inputs
    configuration (13 vector of floats): the current chassis, wheel and joint configuration
    joint_speeds (9 vector of floats): the current joint and wheel speeds
    time_step (float): the time step for numerical integration
    max_speed
    Outputs
    Joint_angles (9 vector of floats): the new joint angles
    delta_theta (9 vector of floats): the change in joint angles
    """
    joint_angles = np.array(configuration[3:12])
    delta_theta = [0] * 9
    for index, joint_angle in enumerate(joint_angles):
        if abs(joint_speeds[index]) <= abs(max_speed):
            joint_speed = joint_speeds[index]
        else:
            joint_speed = math.copysign(max_speed, joint_speeds[index])
            print('speed limited')
        delta_theta[index] = joint_speed * time_step
    joint_angles = joint_angles + delta_theta
    joint_angles = np.concatenate([[0, 0, 0], joint_angles])
    return joint_angles, delta_theta

