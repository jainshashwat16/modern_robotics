import numpy as np
from modern_robotics import MatrixExp6, MatrixLog6, TransInv, FKinSpace
from decimal import Decimal
from simulator import calc_t_sb
import pandas


def trajectory_generator(t_se_i, t_sc_i, t_sc_f, t_ce_grasp, t_ce_standoff, k):
    """Generator the reference trajectory for the end-effector frame using constant twist paths between and fifth order
    time scaling

    Inputs
    t_se_i : The initial configuration of the end-effector in the reference trajectory
    t_sc_i : The cube's initial configuration
    t_sc_f : The cube's desired final configuration
    t_ce_grasp : The ene effector's configuration relative to the cube when it is grasping the cube
    t_ce_standoff : The end-effector's standoff configuration above the cube before and after grasping. This specifies
                    the configuration of the end-effector relative to the cube
    k (int): The number of trajectory reference configurations per .01 seconds

    Outputs
    Trajectory: A list of N configurations of the end-effector along the trajectory. Each item in the list is a 13
        vector of the form:
        11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state
    A .csv file with the entire reference trajectory where each line is:
        r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state
    """

    gripper_state = 0
    dt = Decimal(.01) / k

    standoff_i = np.dot(t_sc_i, t_ce_standoff)  # the first standoff configuration above the cube
    grasp_i = np.dot(t_sc_i, t_ce_grasp)  # the configuration before the first grasp
    grasp_f = np.dot(t_sc_f, t_ce_grasp)  # the configuration when grasping the cube in the final position
    standoff_f = np.dot(t_sc_f, t_ce_standoff)  # the final standoff configuration above the cube

    # move to standoff configuration above cube
    trajectory = np.array(trajectory_segment(t_se_i, standoff_i, 12, dt, gripper_state))

    # move to grasping configuration
    trajectory = np.concatenate((trajectory, np.array(trajectory_segment(standoff_i, grasp_i, 4, dt, gripper_state))))

    # close the gripper
    gripper_state = 1
    trajectory = np.concatenate((trajectory, np.array(move_gripper(grasp_i, gripper_state, 4, dt))))

    # move to standoff configuration
    trajectory = np.concatenate((trajectory, trajectory_segment(grasp_i, standoff_i, 4, dt, gripper_state)))

    # move cube to final standoff position
    trajectory = np.concatenate((trajectory, trajectory_segment(standoff_i, standoff_f, 12, dt, gripper_state)))

    # move cube to final position
    trajectory = np.concatenate((trajectory, trajectory_segment(standoff_f, grasp_f, 4, dt, gripper_state)))

    # open gripper
    gripper_state = 0
    trajectory = np.concatenate((trajectory, move_gripper(grasp_f, gripper_state, 2, dt)))

    # raise the gripper to standoff configuration
    trajectory = np.concatenate((trajectory, trajectory_segment(grasp_f, standoff_f, 2, dt, gripper_state)))

    # write to .csv
    """
    df = pandas.DataFrame(trajectory)
    df.to_csv('trajectory.csv', header=False, index=False)
    """
    return trajectory


def move_gripper(x_start, gripper_state, duration, dt):
    """Returns a series of trajectory segments where the bot doesn't move except for the gripper
    inputs
    x_start (array in SE(3)): the starting frame
    duration (float): the length of the motion in seconds
    dt (float): the length of the timestep in seconds
    gripper_state (bool): the requested new gripper position

    Outputs
    segment : a list of 13 vectors in which the gripper changes from one state to another
    """
    segment = []
    for i in range(0, int(duration / dt)):
        segment += [transform_to_13(x_start, gripper_state)]
    return segment


def trajectory_segment(x_start, x_end, duration, dt, gripper_state):
    """Given the start and end frames in SE(3) returns a constant twist trajectory between the frames
    inputs
    x_start (array in SE(3)): the starting frame
    s_end (array in SE(3): the end frame
    duration (float): the length of the motion in seconds
    dt (float): the length of the timestep in seconds
    gripper_state (bool): the state of the gripper during the motion

    Outputs
    segment :the trajectory between the two points
    """

    segment = []
    s_of_t = time_scaling(duration)

    # calculate the constant portion of the trajectory equation
    log = MatrixLog6(np.dot(TransInv(x_start), x_end))

    for i in range(0, int(duration / dt)):
        # calculate the time dependant portion of the trajectory equation
        x = np.dot(x_start, MatrixExp6(log * s_of_t(i * dt)))
        segment.append(transform_to_13(x, gripper_state))
    return segment


def time_scaling(dt):
    """Returns a fifth order polynomial time scaling function between t_0 and t_end
    Inputs
    dt (float): the duration of the motion

    Outputs
    time_scaling (function): a function that returns an S value for a given t
    """
    # The s, s', s" = 0 at s(0) and s(t):
    equations = np.array([
                            [1,         0,          0,          0,          0,              0],
                            [0,         1,          0,          0,          0,              0],
                            [0,         0,          2,          0,          0,              0],
                            [1,         dt,         dt**2,      dt**3,      dt**4,          dt**5],
                            [0,         1,          2 * dt,     3 * dt**2,  4 * dt**3,      5 * dt**4],
                            [0,         0,          2,          6 * dt,     12 * dt ** 2,   20 * dt ** 3]
                            ])

    b = np.array([0, 0, 0, 1, 0, 0])

    a = np.linalg.solve(equations, b)

    def s_of_t(t):
        t = float(t)
        return a[0] + a[1] * t + a[2] * t ** 2 + a[3] * t ** 3 + a[4] * t ** 4 + a[5] * t ** 5

    return s_of_t


def transform_to_13(x, gripper_state):
    """Takes a transformation matrix and gripper configuration and returns a 13 vector
    Inputs
    x (array in SE(3): the current configuration
    gripper_state (bool): the gripper state
    outputs
    configuration_vector: a 13 vector representing the current configuration
    """
    return [x[0, 0], x[0, 1], x[0, 2], x[1, 0], x[1, 1], x[1, 2], x[2, 0], x[2, 1], x[2, 2], x[0, 3], x[1, 3], x[2, 3], gripper_state]


def thirteen_to_transform(x):
    """Takes a 13 vector specifying the end effector configuration and returns a transformation
    Inputs
    x (13 vector of floats): the end effector coordinates and gripper configuration
    Output
    t (array in SE3): the transformation matrix for the end effector"""
    t = np.array([[x[0],    x[1],   x[2],   x[9]],
                 [x[3],     x[4],   x[5],   x[10]],
                  [x[6],    x[7],   x[8],   x[11]],
                  [0,       0,      0,      1]])
    return t

