import gym.spaces
import numpy as np
import rocket_lander_gym
from pidcontroller import *
import math
import matplotlib.pyplot as plt


def discretize_actions(action, bins_num=6, min_val=-1.0, max_val=1.0):
    """Discretize action into one of the specified bins."""
    intervals = np.linspace(min_val, max_val, bins_num)

    # digitize returns the index of action in intervals
    return intervals[np.digitize(action, intervals) - 1]


def plot_response(data_dict):
    """plot the responses of the rocket while landing"""
    plt.figure(figsize=(12, 6))
    for key, value in data_dict.items():
        plt.plot(value, label=key)
    plt.legend()
    plt.grid()
    plt.ylim(-1.1, 1.1)
    plt.title('2-DoF PID Control')
    plt.ylabel('Value')
    plt.xlabel('Steps')
    plt.show()


def run_simulation(env, pid_y_controller, pid_theta_controller):
    """ Runs the simulation and collects data for plotting. """
    largest_reward = 0  # Dummy variable for checking whether we landed successfully or not
    ACTION_X = 0  # !Setting the throttle's gimbal DoF to 0 permanently!
    data = {'x': [], 'y': [], 'theta': [], 'vx': [], 'vy': [], 'omega': []}
    action_set = [0, 0, 0]  # Initial action

    while True:
        env.render()

        observation, reward, done, _ = env.step(action_set)

        # append state variables for plotting
        # 0,1,2,7,8,9 corresponds to the specified variables x,y,theta,vx,vy,omega respectively
        data['x'].append(observation[0])
        data['y'].append(observation[1])
        data['theta'].append(observation[2])
        data['vx'].append(observation[7])
        data['vy'].append(observation[8])
        data['omega'].append(observation[9])

        # update the action values using the PID's feedback update
        action_y = np.clip(pid_y_controller.update(
            [observation[1], observation[8]], abs(observation[0]) + SETPOINTS['y']), -1.0, 1.0)

        action_theta = np.clip(pid_theta_controller.update([observation[2], observation[9]],
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
    return data


# Setpoints for the PID controllers.
SETPOINTS = {'y': -1.05, 'theta': 0}

# Create the environment and controllers.
env = gym.make('RocketLander-v0')
env.reset()
y_controller = PIDy(15000, 10, 5000, SETPOINTS['y'])
theta_controller = PIDtheta(1000, 2.5, 750, SETPOINTS['theta'])


# Run the simulation and plot the results.
data = run_simulation(env, y_controller, theta_controller)
plot_response(data)