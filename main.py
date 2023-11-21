import gym.spaces
from pidcontroller import *
from simulation import run_simulation
from functions import plot_response
import rocket_lander_gym

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
