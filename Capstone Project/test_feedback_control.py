from modern_robotics import JacobianBody, TransInv, Adjoint, FKinSpace
from feedback_control import Controller
import numpy as np
import pytest
from utils import to_decimal

names = "k_p, k_i, V_expected, u_expected"
V_expected_1 = np.array([0, 0, 0, 21.409, 0, 6.455])
u_expected_1 = np.array([157.5, 157.5, 157.5, 157.5, 0, -654.3, 1400.9, -746.8, 0])

V_expected_2 = np.array([0, .171, 0, 21.488, 0, 6.562])
u_expected_2 = np.array([157.5, 157.5, 157.5, 157.5, 0, -654.3, 1400.9, -746.8, 0])

values = [(np.zeros((6, 6)), np.zeros((6, 6)), V_expected_1, u_expected_1),
          (np.eye(6), np.eye(6), V_expected_2, u_expected_2)]


@pytest.mark.parametrize(names, values)
def test_feedback_control_v(k_p, k_i, V_expected, u_expected):

    # verified
    x_d = np.array(to_decimal([[0, 0, 1, .5],
                    [0, 1, 0, 0],
                    [-1, 0, 0, .5],
                   [0, 0, 0, 1]]))

    x_d_next = np.array(to_decimal([[0, 0, 1, .6],
                         [0, 1, 0, 0],
                         [-1, 0, 0, .3],
                         [0, 0, 0, 1]]))

    x = np.array(to_decimal([[.170, 0, .985, .387],
                  [0, 1, 0, 0],
                  [-.985, 0, .170, .570],
                  [0, 0, 0, 1]]))

    configuration = [0, 0, 0, 0, 0, 0, 0, 0, .2, -1.6, 0, 0, 0]
    configuration = to_decimal(configuration)

    t_b0 = np.array(to_decimal([[1, 0, 0, .1662],
                     [0, 1, 0, 0],
                     [0, 0, 1, .0026],
                     [0, 0, 0, 1]]))

    M = np.array(to_decimal([[1, 0, 0, .033],
                  [0, 1, 0, 0],
                  [0, 0, 1, .6546],
                  [0, 0, 0, 1]]))

    b_list = np.array(to_decimal([[0, 0, 1, 0, .033, 0],
                       [0, -1, 0, -.5076, 0, 0],
                       [0, -1, 0, -.3526, 0, 0],
                       [0, -1, 0, -.2176, 0, 0],
                       [0, 0, 1, 0, 0, 0]])).T

    controller = Controller(k_p, k_i, .01)
    # end verified

    v, x_err = controller.feedback_control(x_d, x_d_next, x)
    u, x_err = controller.commanded_joint_vels(x_d, x_d_next, x, configuration, t_b0, M, b_list)

    print(v)
    print(u)
    assert np.allclose(v, V_expected, rtol=1e-4)


@pytest.mark.parametrize(names, values)
def test_feedback_control_v(k_p, k_i, V_expected, u_expected):

    # verified
    x_d = np.array(to_decimal([[0, 0, 1, .5],
                    [0, 1, 0, 0],
                    [-1, 0, 0, .5],
                   [0, 0, 0, 1]]))

    x_d_next = np.array(to_decimal([[0, 0, 1, .6],
                         [0, 1, 0, 0],
                         [-1, 0, 0, .3],
                         [0, 0, 0, 1]]))

    x = np.array(to_decimal([[.170, 0, .985, .387],
                  [0, 1, 0, 0],
                  [-.985, 0, .170, .570],
                  [0, 0, 0, 1]]))

    configuration = [0, 0, 0, 0, 0, 0, 0, 0, .2, -1.6, 0, 0, 0]
    configuration = to_decimal(configuration)

    t_b0 = np.array(to_decimal([[1, 0, 0, .1662],
                     [0, 1, 0, 0],
                     [0, 0, 1, .0026],
                     [0, 0, 0, 1]]))

    M = np.array(to_decimal([[1, 0, 0, .033],
                  [0, 1, 0, 0],
                  [0, 0, 1, .6546],
                  [0, 0, 0, 1]]))

    b_list = np.array(to_decimal([[0, 0, 1, 0, .033, 0],
                       [0, -1, 0, -.5076, 0, 0],
                       [0, -1, 0, -.3526, 0, 0],
                       [0, -1, 0, -.2176, 0, 0],
                       [0, 0, 1, 0, 0, 0]])).T

    controller = Controller(k_p, k_i, .01)
    # end verified

    v, x_err = controller.feedback_control(x_d, x_d_next, x)
    u, x_err = controller.commanded_joint_vels(x_d, x_d_next, x, configuration, t_b0, M, b_list)

    print(v)
    print(u)
    assert np.allclose(u, u_expected, rtol=1e-2)