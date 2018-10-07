from modern_robotics import MatrixLog6, TransInv, se3ToVec, Adjoint, FKinBody, JacobianBody
from utils import calc_f_matrix, calc_t_sb
import numpy as np


class Controller:
    def __init__(self, k_p, k_i, dt):
        """Initialized the controller class
        Input
        k_p (array): proportional gain matrix
        k_i (array): integral gain matrix
        dt (float): the timestep between reference configurations
        Output
        A feedback controller object with the specified parameters
        """
        self.x_err_integral = np.array([0, 0, 0, 0, 0, 0])
        self.k_p = k_p
        self.k_i = k_i
        self.dt = dt

    def feedback_control(self, x_d, x_d_next, x):
        """Returns a commanded end effector twist in the end effector frame given the current configuration, reference
        configurations and gains
        Input
        x_d (array in SE(3)): The current end effector reference configuration
        x_d_next (array in SE(3): the end effector reference configuration at the next timestep in the reference trajectory
        x (array in SE(3)): the actual end effector configuration

        Output
        v: the end effector twist in the end-effector frame
        """

        x_err = se3ToVec(MatrixLog6(np.dot(TransInv(x), x_d)))

        # calculate feedforward reference twist
        v_d = se3ToVec((1 / self.dt) * MatrixLog6(np.dot(TransInv(x_d), x_d_next)))
        print(f' x_err {x_err}')
        print(v_d)

        v = np.dot(Adjoint(np.dot(TransInv(x), x_d)), v_d) + np.dot(self.k_p, x_err) + np.dot(self.k_i, self.x_err_integral)

        self.x_err_integral = np.add(self.x_err_integral, x_err)

        return v, x_err

    def commanded_joint_vels(self, x_d, x_d_next, x, configuration, t_b0, M, b_list):
        """Calculates a commanded end effector twist in the end effector frame given the current configuration, reference
        configurations and gains. Returns the joint velocities required to achieve this twist
        Inputs

        Output
        """
        theta = configuration[3:11]
        v, x_err = self.feedback_control(x_d, x_d_next, x)

        t_0e = FKinBody(M, b_list, theta[3:])

        f = calc_f_matrix()
        f6 = np.zeros((6, 4))
        f6[2:5, 0:4] = f

        j_base = np.dot(Adjoint(np.dot(TransInv(t_0e), TransInv(t_b0))), f6)

        j_body = JacobianBody(b_list, theta[3:8])

        je = np.concatenate((j_base, j_body), axis=1)
        print(je)
        u = np.dot(np.linalg.pinv(je, 1e-3), v)
        return u, x_err

