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
    thirteen = [1, 0, 0, 0, 1, 0, 0, 0, 1, 2, 3, 4, 1]
    transform_expected = [[1, 0, 0, 2],
                          [0, 1, 0, 3],
                          [0, 0, 1, 4],
                          [0, 0, 0, 1]]
    transform = trajectory_generator.thirteen_to_transform(thirteen)
    assert np.allclose(transform, transform_expected)


def test_transform_to_13():
    transform = np.array([[1, 0, 0, 2],
                        [0, 1, 0, 3],
                        [0, 0, 1, 4],
                        [0, 0, 0, 1]])
    thirteen_expected = [1, 0, 0, 0, 1, 0, 0, 0, 1, 2, 3, 4, 1]
    thirteen = trajectory_generator.transform_to_13(transform, 1)
    assert np.allclose(thirteen, thirteen_expected)