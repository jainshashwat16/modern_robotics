from trajectory_generator import trajectory_generator, thirteen_to_transform
from feedback_control import Controller
from simulator import next_state
import numpy as np
import pandas
from utils import calc_t_se, to_decimal

t_sc_i = np.array([[1, 0, 0, 1],
                   [0, 1, 0, 0,],
                   [0, 0, 1, .025],
                   [0, 0, 0, 1]])

t_sc_f = np.array([[0, 1, 0, 0],
                   [-1, 0, 0, -1],
                   [0, 0, 1, .025],
                   [0, 0, 0, 1]])

t_se_i = np.array([[0, 0, 1, 0],
                   [0, 1, 0, 0],
                   [-1, 0, 0, .5],
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

print(b_list.dtype)

reference_trajectory = trajectory_generator(t_se_i, t_sc_i, t_sc_f, t_ce_grasp, t_ce_standoff, 1)
dt = .01
print(type(dt))
max_speed = 100
k = 1

k_p = np.eye(6) * 0
k_i = np.eye(6) * 0

feedback_controller = Controller(k_p, k_i, dt)
trajectory = []
err = []
configuration = [0, 0, 0, -.1, -.1, -.1, -.1, -.1, 0, 0, 0, 0, 0]
u = [0, 0, 0, 0, 0, 0, 0, 0, 0]

u_list = []
df = pandas.DataFrame(reference_trajectory)
df.to_csv('reference_trajectory.csv', header=False, index=False)

for index, reference_configuration in enumerate(reference_trajectory):
    if index == len(reference_trajectory) - 1:
        break
    x_d = thirteen_to_transform(reference_configuration)
    x_d_next = thirteen_to_transform(reference_trajectory[index + 1])

    configuration = next_state(configuration, u, dt, max_speed)
    configuration = np.append(configuration, reference_configuration[12])

    t_se = calc_t_se(configuration, M, b_list, t_b0)
    u, x_err = feedback_controller.commanded_joint_vels(x_d, x_d_next, t_se, configuration, t_b0, M, b_list)
    print(f'u {u}')
    print(configuration)

    if index % k == 0:
        trajectory.append(configuration)
        err.append(x_err)
        u_list.append(u)

df = pandas.DataFrame(trajectory)
df.to_csv('trajectory.csv', header=False, index=False)
df = pandas.DataFrame(err)
df.to_csv('err.csv', header=False, index=False)
df = pandas.DataFrame(u_list)
df.to_csv('u_list.csv', header=False, index=False)
