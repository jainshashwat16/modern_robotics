import pytest
import simulator

names = "configuration, joint_speeds, time_step, max_speed, end_config"
initial_config = [0] * 12
js_1 = [0, 0, 0, 0, 0, 10, 10, 10, 10]
js_2 = [0, 0, 0, 0, 0, -10, 10, -10, 10]
js_3 = [0, 0, 0, 0, 0, -10, 10, 10, -10]
end_config_1 = [0, .475, 0, 0, 0, 0, 0, 0, 10, 10, 10, 10]
end_config_2 = [0, 0, .475, 0, 0, 0, 0, 0, -10, 10, -10, 10]
end_config_3 = [1.234, 0, 0, 0, 0, 0, 0, 0, -10, 10, 10, -10]

values = [(initial_config, js_1, .01, 20, end_config_1),
          (initial_config, js_2, .01, 20, end_config_2),
          (initial_config, js_3, .01, 20, end_config_3)]

euler_end_1 = [0, 0, 0, 0, 0, 0, 0, 0, 10, 10, 10, 10]
euler_end_2 = [0, 0, 0, 0, 0, 0, 0, 0, -10, 10, -10, 10]
euler_end_3 = [0, 0, 0, 0, 0, 0, 0, 0, -10, 10, 10, -10]

euler_values = [(initial_config, js_1, .1, 20, euler_end_1),
                (initial_config, js_2, .1, 20, euler_end_2),
                (initial_config, js_3, .1, 20, euler_end_3)]


@pytest.mark.parametrize(names, values)
def test_next_state(configuration, joint_speeds, time_step, max_speed, end_config):

    for i in range(0, int(1 / time_step)):
        configuration = simulator.next_state(configuration, joint_speeds, time_step, max_speed)
    assert configuration == pytest.approx(end_config, abs=1e-3)


@pytest.mark.parametrize(names, euler_values)
def test_euler_step(configuration, joint_speeds, time_step, max_speed, end_config):
    for i in range(0, int(1 / time_step)):
        configuration, delta_theta = simulator.euler_step(configuration, joint_speeds, time_step, max_speed)
    assert configuration == pytest.approx(end_config, abs=1e-3)
