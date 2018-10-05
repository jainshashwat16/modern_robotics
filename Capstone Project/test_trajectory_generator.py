import pytest
import trajectory_generator
import numpy as np

names = "duration, test_point, expected"
values = [(1, 0, 0), (1, 1, 1), (2, 0, 0), (2, 2, 1)]


@pytest.mark.parametrize(names, values)
def test_time_scale(duration, test_point, expected):
    s_of_t = trajectory_generator.time_scaling(duration)
    assert s_of_t(test_point) == pytest.approx(expected)


def test_thirteen_to_transform():

    configuration = [-0.75959, -0.47352, 0.058167, 0.80405, -0.91639, -0.011436, 0.054333, 0.00535, 1.506, -1.3338, 1.5582, 1.6136, 0]

    transform = trajectory_generator.thirteen_to_transform(configuration)
    thirteen = trajectory_generator.transform_to_13(transform, 0)
    print(transform)
    print(thirteen)
    assert np.allclose(configuration, thirteen)


def test_configuration_to_transform():
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

    transform = trajectory_generator.configuration_to_transform([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], t_b0, M, b_list)
    print(transform)
    expected = np.dot(t_b0, M)
    expected[2, 3] += .0963
    print(expected)
    assert np.allclose(expected, transform)