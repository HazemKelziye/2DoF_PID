import numpy as np
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