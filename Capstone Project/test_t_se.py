import pytest
from utils import calc_t_se
import numpy as np

t_b0 = np.array([[1, 0, 0, .1662],
                 [0, 1, 0, 0],
                 [0, 0, 1, .0026],
                 [0, 0, 0, 1]])

M = np.array([[1, 0, 0, .033],
              [0, 1, 0, 0],
              [0, 0, 1, .6546],
              [0, 0, 0, 1]])

b_list = np.array([[0, 0, 1, 0, .033, 0],
                   [0, -1, 0, -.5076, 0, 0],
                   [0, -1, 0, -.3526, 0, 0],
                   [0, -1, 0, -.2176, 0, 0],
                   [0, 0, 1, 0, 0, 0]]).T

t_sb_i = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, .0963],
                   [0, 0, 0, 1]])

# test case one all joints zero
configuration_1 = [0] * 13
t_s0 = np.dot(t_sb_i, t_b0)
expected_t_se_1 = np.dot(t_s0, M)

# test case 2 base 1 m forward
configuration_2 = [0] * 13
configuration_2[1] = 1
expected_t_se_2 = np.dot(t_s0, M)
expected_t_se_2[0, 3] += 1

# test case 3 joint 1 90 degrees
configuration_3 = [0] * 13
configuration_3[3] = np.pi / 2
expected_t_se_3 = np.array([[0, -1, 0, expected_t_se_1[0, 3] - .033],
                            [1, 0, 0, .033],
                            [0, 0, 1, expected_t_se_1[2, 3]],
                            [0, 0, 0, 1]])

# test case 4 chassis 90 degrees
configuration_4 = [0] * 13
configuration_4[0] = np.pi / 2
expected_t_se_4 = np.array([[0, -1, 0, 0],
                            [1, 0, 0, expected_t_se_1[0, 3]],
                            [0, 0, 1, expected_t_se_1[2, 3]],
                            [0, 0, 0, 1]])

names = "configuration, M, b_list, t_b0, expected_t_se"
values = [(configuration_1, M, b_list, t_b0, expected_t_se_1),
          (configuration_2, M, b_list, t_b0, expected_t_se_2),
          (configuration_3, M, b_list, t_b0, expected_t_se_3),
          (configuration_4, M, b_list, t_b0, expected_t_se_4)]


@pytest.mark.parametrize(names, values)
def test_t_se(configuration, M, b_list, t_b0, expected_t_se):
    t_se = calc_t_se(configuration, M, b_list, t_b0)
    print(configuration)
    print(t_se)
    print(expected_t_se)
    assert np.allclose(t_se, expected_t_se, atol=1e-4)
