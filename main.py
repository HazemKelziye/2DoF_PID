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

EPISODES_NUMBER = 10
SETPOINTS = [0, -1.05, 0]  # Setpoints/desired-points for optimizing the PID controller
# ep_counter = 1
episodes = {}
largest_reward = 0  # Dummy var for checking whether we landed successfully or not

# For storing the state-action pairs
sa_pairs = []

ACTION_X = 0  # !Setting the throttle's gimbal DoF to 0 permanently!
action_y = 0
action_theta = 0

# Initializing the state-space's lists
x_pos_data, y_pos_data, orient_data, vx_data, vy_data, omega_data = [], [], [], [], [], []

# Initialization for our PID controllers with their Gains
y_controller = PIDy(15000, 10, 5000, SETPOINTS[1])
theta_controller = PIDtheta(1000, 2.5, 750, SETPOINTS[2])

env = gym.make('RocketLander-v0')

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
            [observation[1], observation[8]], abs(observation[0]) + SETPOINTS[1]), -1, 1)

        action_theta = np.clip(theta_controller.update([observation[2], observation[9]],
                                                       (math.pi / 4) * (observation[0] + observation[7])), -1, 1)

        action_set = np.array([ACTION_X, action_y, action_theta])

        # Adding the S-A pairs, the state-space's dimension is 9 and the action-space's dimension is 2
        sa_pairs.append([list(observation)[:6] + list(observation[7:]), list(action_set)[1:]])  # Adding the (s,a) pairs

        # Making a scheme for learning whether the landing was successful or not
        largest_reward = reward if reward > largest_reward else largest_reward

        if done:
            # Deciding whether the landing was successful or not
            success = True if largest_reward >= 0.1 else False
            print(f"Simulation {ep_counter} done : {success}.")
            # Making a dictionary to store the episodic Dataset in a JSON file
            episodes[f"episode{ep_counter}"] = {
                "SA_pairs": sa_pairs,
                "success": success,
            }
            break

    env.close()


# json_object = json.dumps(episodes)
# with open("episodes.json", "w") as outfile:
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
