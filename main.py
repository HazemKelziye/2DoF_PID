"""
Created on Tue Oct 26 09:34:00 2023

@author: Hazem
"""
# import gym
import gym.spaces
import numpy as np
import rocket_lander_gym
from pidcontroller import *
import math
import matplotlib.pyplot as plt
import json

EPISODES_NUMBER = 1
SETPOINTS = [0, -1.05, 0]  # Setpoints/desired-points for optimizing the PID controller
episodes = {}
largest_reward = 0  # Dummy var for checking whether we landed successfully or not

resulting_pattern = []  # add the 3 state 1 action pairs
dataset = []  # aggregate the 3s,1a pairs

sa_pairs = []  # For storing the state-action pairs

ACTION_X = 0  # !Setting the throttle's gimbal DoF to 0 permanently!
action_y = 0
action_theta = 0

# Initialize the state-space's lists
x_pos_data, y_pos_data, orient_data, vx_data, vy_data, omega_data = [], [], [], [], [], []

# Initialization for our PID controllers with their Gains
y_controller = PIDy(15000, 10, 5000, SETPOINTS[1])
theta_controller = PIDtheta(1000, 2.5, 750, SETPOINTS[2])

env = gym.make('RocketLander-v0')


def discretize_actions(action):
    """discretize given action, according to specified bins_num"""
    bins_num = 5
    min_val = -1.0
    max_val = 1.0
    # bin_width = (max_val - min_val) / bins_num
    intervals = np.linspace(min_val, max_val, bins_num + 1)

    for i in range(bins_num):
        if action < intervals[i + 1]:
            return intervals[i]

    return intervals[-1] if action >= intervals[-1] else None


for ep_counter in range(1, EPISODES_NUMBER + 1):
    env.reset()

    action_set = env.action_space.sample()
    observation, _, done, _ = env.step(action_set)

    while True:
        env.render()
        observation, reward, done, _ = env.step(action_set)

        # Appending the state variables
        x_pos_data.append(observation[0])
        y_pos_data.append(observation[1])
        orient_data.append(observation[2])
        vx_data.append(observation[7])
        vy_data.append(observation[8])
        omega_data.append(observation[9])

        # Taking actions w.r.t. the PID controller's feedback, If one of the legs contacts the ground i.e.
        # MISSION_ACCOMPLISHED==1 set action_y = 0 (kill off throttle engine)
        action_y = np.clip(y_controller.update(
            [observation[1], observation[8]], abs(observation[0]) + SETPOINTS[1]), -1.0, 1.0)

        action_theta = np.clip(theta_controller.update([observation[2], observation[9]],
                                                       (math.pi / 4) * (observation[0] + observation[7])),
                               -1, 1) if largest_reward <= 0 else 0

        # Discretizing the input actions y and theta
        action_set = np.array([ACTION_X, discretize_actions(action_y), discretize_actions(action_theta)])

        # Adding the S-A pairs, the state-space's dimension is 5 and the action-space's dimension is 1

        sa_pairs.append([[list(observation)[0]
                         , list(observation)[1]
                         , list(observation)[2]
                         , list(observation)[7]
                         , list(observation)[8]], list(action_set)[1:]])  # Adding the (s,a) pairs


        # Making a scheme for learning whether the landing was successful or not
        largest_reward = reward if reward > largest_reward else largest_reward

        if done:
            # Deciding whether the landing was successful or not
            success = True if largest_reward >= 0.05 else False
            print(f"Simulation {ep_counter} done : {success}.")
            # # Making a dictionary to store the episodic Dataset in a JSON file
            # for t in range(len(sa_pairs) - 2):
            #     state_0, action_0 = sa_pairs[t]
            #     state_1, action_1 = sa_pairs[t + 1]
            #     state_2, action_2 = sa_pairs[t + 2]
            #
            #     sub_pattern = [state_0 + state_1 + state_2, action_2]
            #     resulting_pattern.append(sub_pattern)
            #
            # dataset += resulting_pattern

            resulting_pattern.extend(sa_pairs)

            break

    env.close()

# print(sa_pairs)
# json_object = json.dumps(resulting_pattern)
# with open("new_50_episodes2.json", "w") as outfile:
#     outfile.write(json_object)


# Function for plotting the response of the system
def plot_response():
    plt.plot(x_pos_data, label="x")
    plt.plot(y_pos_data, label="y")
    plt.plot(orient_data, label="theta")
    plt.plot(vx_data, label="vx")
    plt.plot(vy_data, label="vy")
    plt.plot(omega_data, label="omega")
    plt.legend()
    plt.grid()
    plt.ylim(-1.1, 1.1)
    plt.title('2-DoF PID Control')
    plt.ylabel('Value')
    plt.xlabel('Steps')
    plt.show()

# plot_response()  # Plotting the response of the system
