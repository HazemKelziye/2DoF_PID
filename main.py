import gym.spaces
import numpy as np
import rocket_lander_gym
from pidcontroller import *
import math
import matplotlib.pyplot as plt


def discretize_actions(action):
    """discretize given action, according to specified bins_num"""
    bins_num = 6
    min_val = -1.0
    max_val = 1.0
    # bin_width = (max_val - min_val) / bins_num
    intervals = np.linspace(min_val, max_val, bins_num)

    for i in range(bins_num):
        if action <= intervals[i]:
            return intervals[i]

    return intervals[-1] if action >= intervals[-1] else None


def plot_response(*args):
    """plot the responses of the rocket while landing"""
    plt.plot(args[0], label="x")
    plt.plot(args[1], label="y")
    plt.plot(args[2], label="theta")
    plt.plot(args[3], label="vx")
    plt.plot(args[4], label="vy")
    plt.plot(args[5], label="omega")
    plt.legend()
    plt.grid()
    plt.ylim(-1.1, 1.1)
    plt.title('2-DoF PID Control')
    plt.ylabel('Value')
    plt.xlabel('Steps')
    plt.show()


SETPOINTS = [0, -1.05, 0]  # Setpoints/desired-points for optimizing the PID controller
largest_reward = 0  # Dummy variable for checking whether we landed successfully or not

ACTION_X = 0  # !Setting the throttle's gimbal DoF to 0 permanently!
action_y = -0.2
action_theta = 0

# Initialize the state-space's lists for plotting
x_pos_data, y_pos_data, orient_data, vx_data, vy_data, omega_data = [], [], [], [], [], []

# Initialization for our PID controllers with their Gains
y_controller = PIDy(15000, 10, 5000, SETPOINTS[1])
theta_controller = PIDtheta(1000, 2.5, 750, SETPOINTS[2])

env = gym.make('RocketLander-v0')
env.reset()

action_set = env.action_space.sample()

while True:
    env.render()
    observation, reward, done, _ = env.step(action_set)

    # append state variables for plotting
    x_pos_data.append(observation[0])
    y_pos_data.append(observation[1])
    orient_data.append(observation[2])
    vx_data.append(observation[7])
    vy_data.append(observation[8])
    omega_data.append(observation[9])

    # update the action values using the PID's feedback update
    action_y = np.clip(y_controller.update(
        [observation[1], observation[8]], abs(observation[0]) + SETPOINTS[1]), -1.0, 1.0)

    action_theta = np.clip(theta_controller.update([observation[2], observation[9]],
                                                   (math.pi / 4) * (observation[0] + observation[7])), -1, 1)

    # Pass the discretized actions to action_set
    action_set = np.array([ACTION_X, discretize_actions(action_y), discretize_actions(action_theta)])

    # Making a scheme for learning whether the landing was successful or not
    largest_reward = reward if reward > largest_reward else largest_reward
    if done:
        # Deciding whether the landing was successful or not
        success = True if largest_reward >= 0.05 else False
        print(f"Simulation is done : {success}.")
        break

env.close()

plot_response(x_pos_data, y_pos_data, orient_data, vx_data, vy_data, omega_data)  # Plotting the response of the system
