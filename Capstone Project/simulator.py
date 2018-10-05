import numpy as np
import math
import modern_robotics


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

    V_b = np.dot(F, delta_theta[5:])

    V_b6 = np.array([0, 0, V_b[0], V_b[1], V_b[2], 0])

    T_sb_k = calc_t_sb(configuration[0], configuration[1], configuration[2], .0963)

    T_sb_k_plus_1 = np.dot(T_sb_k, modern_robotics.MatrixExp6(modern_robotics.VecTose3(V_b6)))
    q_k_plus_1 = modern_robotics.se3ToVec(T_sb_k_plus_1)

    q_k_plus_1 = q_k_plus_1[2:5]
    new_configuration = np.concatenate([q_k_plus_1, np.array(joint_angles[3:])])

    """ I found that the rotation value resulting from the above calculations deviates from the expected value 
    significantly with successive iteration. Incrementing the value below is my solution"""
    new_configuration[0] = configuration[0] + V_b[0]
    return new_configuration


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
    Values are hard coded due to floating point error issues when calculating them
    Returns
    F (array of floats): the chassis F matrix
    """
    wheel_rad = .0475
    F = np.array([  [-2.5974025974025974, 2.5974025974025974, 2.5974025974025974, -2.5974025974025974],
                    [1, 1, 1, 1],
                    [-1, 1, -1, 1]])
    F = wheel_rad * .25 * F
    return F


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
        print(f'joing speed {joint_speeds[index]}')
        if abs(joint_speeds[index]) <= abs(max_speed):
            joint_speed = joint_speeds[index]
        else:
            joint_speed = math.copysign(max_speed, joint_speeds[index])
        delta_theta[index] = joint_speed * time_step
    joint_angles = joint_angles + delta_theta
    joint_angles = np.concatenate([[0, 0, 0], joint_angles])
    return joint_angles, delta_theta

