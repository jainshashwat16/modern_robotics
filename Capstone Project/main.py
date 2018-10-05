from trajectory_generator import trajectory_generator, thirteen_to_transform, transform_to_13, configuration_to_transform
from feedback_control import Controller
from simulator import next_state
import numpy as np
import pandas

t_sc_i = np.array([[1, 0, 0, 1],
                   [0, 1, 0, 0,],
                   [0, 0, 1, .025],
                   [0, 0, 0, 1]])

t_sc_f = np.array([[0, 1, 0, 0],
                   [-1, 0, 0, -1],
                   [0, 0, 1, .025],
                   [0, 0, 0, 1]])

t_se_i = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 1],
                   [0, 0, 0, 1]])

t_ce_grasp = np.array([[0, 0, 1, 0],
                       [0, 1, 0, 0],
                       [-1, 0, 0, 0],
                       [0, 0, 0, 1]])

t_ce_standoff = np.array([[0, 0, 1, 0],
                          [0, 1, 0, 0],
                          [-1, 0, 0, .1],
                          [0, 0, 0, 1]])

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

reference_trajectory = trajectory_generator(t_se_i, t_sc_i, t_sc_f, t_ce_grasp, t_ce_standoff, 1)
dt = .01
max_speed = 5
k = 1

k_p = np.eye(6) * 0
k_i = np.eye(6) * 0

feedback_controller = Controller(k_p, k_i, dt)
trajectory = []
configuration = transform_to_13(t_se_i, 0)

df = pandas.DataFrame(reference_trajectory)
df.to_csv('reference_trajectory.csv', header=False, index=False)

for index, reference_configuration in enumerate(reference_trajectory):
    if index == len(reference_trajectory) - 1:
        break
    x_d = thirteen_to_transform(reference_configuration)
    x_d_next = thirteen_to_transform(reference_trajectory[index + 1])
    x = configuration_to_transform(configuration, t_b0, M, b_list)
    print(f'x {x}')
    u = feedback_controller.commanded_joint_vels(x_d, x_d_next, x, configuration, t_b0, M, b_list)
    print(f'u {u}')
    print(configuration)
    configuration = next_state(configuration, u, dt, max_speed)
    if index % k == 0:
        trajectory.append(configuration)


df = pandas.DataFrame(trajectory)
df.to_csv('trajectory.csv', header=False, index=False)
