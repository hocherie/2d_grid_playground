import numpy as np
import matplotlib.pyplot as plt
from ecbf_quad_plotone import plot_h
import seaborn as sns
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
sns.set()
sns.set_style("ticks")

# plt.style.use('fivethirtyeight')

def plot_u():
    
    u_trial_nom_x_nonoise = np.load("u_trial_nom_x_nonoise.npy")
    u_trial_nom_y_nonoise = np.load("u_trial_nom_y_nonoise.npy")
    u_trial_safe_x_nonoise = np.load("u_trial_safe_x_nonoise.npy")
    u_trial_safe_y_nonoise = np.load("u_trial_safe_y_nonoise.npy")
    num_it = u_trial_safe_y_nonoise.shape[0]
    ax1 = plt.subplot(2, 1, 1)
    ax1.scatter(np.arange(num_it)*0.1, u_trial_safe_x_nonoise[:, 0],s=1)
    ax1.scatter(np.arange(num_it)*0.1,u_trial_nom_x_nonoise[:,0],s=1)
    ax1.set_ylabel("x accel. (m/s^2)")
    ax1.set_xlim([0,400])
    ax1.legend(["Safe", "Nominal"], markerscale=6)
    ax1.set_title("u (No Noise)")
    ax2 = plt.subplot(2, 1, 2)
    ax2.scatter(np.arange(num_it)*0.1, u_trial_safe_y_nonoise[:, 0],s=1)
    ax2.scatter(np.arange(num_it)*0.1, u_trial_nom_y_nonoise[:, 0],s=1)
    ax2.set_xlabel("Time (s)")
    ax2.set_xlim([0, 400])
    ax2.set_ylabel("y accel. (m/s^2)")
    
    
    plt.show()
def plot_robot_traj():
    state_hist_x_trial = np.load("state_hist_x_trial_nonoise.npy")
    state_hist_y_trial = np.load("state_hist_y_trial_nonoise.npy")
    # Plot robot trajectories
    plt.figure()
    obs_loc = [0, 0]
    plot_h(np.atleast_2d(obs_loc).T) #plot safe set
    plt.plot(state_hist_x_trial, state_hist_y_trial, linewidth=2)
    start_x_list = [1.5, 1.5, -1, -2, -4, 2.6]
    start_y_list = [4, -2, -3, 4.7, -2, 0.8]
    goal_x_list = [-2, 1.5, 0, 0.6, 2.7, -3.9]
    goal_y_list = [-4, 4.5, 4.5, -3.9, 0.5, -0.6]
    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
    for i in range(len(start_x_list)): # plot all starts
        plt.scatter(start_x_list[i], start_y_list[i], marker="o",c=colors[i])
    for i in range(len(start_x_list)):  # plot all ends
        plt.scatter(goal_x_list[i], goal_y_list[i], marker="*", c=colors[i])
    plt.axis("equal")
    plt.grid(False)
    plt.title("Robot Trajectories (No Noise)")

    # Make Custom legend
    legend_elements = [Line2D([0], [0], color='k', lw=5, label='Obstacle'),
                       Line2D([0], [0], marker='o', color='w', label='Start',
                              markerfacecolor='k', markersize=10),
                       Line2D([0], [0], marker='*', color='w', label='Goal',
                              markerfacecolor='k', markersize=10)
                       ]

    plt.legend(handles=legend_elements)
# Create the figure
# fig, ax = plt.subplots()
# ax.legend(handles=legend_elements, loc='center')
    plt.show()


def plot_h_onetrial():
    fig = plt.figure(constrained_layout=True)
    gs = fig.add_gridspec(5,1)
    h_trial = np.load("h_trial_nonoise.npy")

    num_it = h_trial.shape[0]
    
    ax1 = fig.add_subplot(gs[:3,:])
    # Plot metrics over time
    ax1.set_title("h (No Noise)")
    ax1.plot(np.arange(num_it)*0.1, h_trial)
    ax1.plot(np.arange(num_it)*0.1, np.zeros_like(range(num_it)), 'k--')
    ax1.set_ylabel("h (zoomed in)")
    ax2 = fig.add_subplot(gs[3:, :])
    ax2.plot(np.arange(num_it)*0.1, h_trial)
    ax2.plot(np.arange(num_it)*0.1, np.zeros_like(range(num_it)), 'k--')
    ax2.set_ylim([-0.005,0.05])
    ax2.set_ylabel("h (zoomed in)")
    plt.xlabel("Time (s)")
    
    
# plt.ylim((-5, 5))  # highlight if violate safety

    plt.show()


def plot_h_trial():
    h_trial_mean_list = np.load("h_trial_mean_list.npy")
    h_trial_var_list = np.load("h_trial_var_list.npy")

    num_it = h_trial_var_list.shape[0]

    # Plot metrics over time
    plt.fill_between(range(num_it), h_trial_mean_list[:, 2] -
                    h_trial_var_list[:, 2], h_trial_mean_list[:, 2]+h_trial_var_list[:, 2], color='green', alpha=0.2)
    plt.fill_between(range(num_it), h_trial_mean_list[:, 1] -
                    h_trial_var_list[:, 1], h_trial_mean_list[:, 1]+h_trial_var_list[:, 1], color='orange', alpha=0.2)
    plt.fill_between(range(num_it), h_trial_mean_list[:, 0] -
                    h_trial_var_list[:, 0], h_trial_mean_list[:, 0]+h_trial_var_list[:, 0], color='blue', alpha=0.2)
    plt.plot(range(num_it), h_trial_mean_list)
    plt.plot(range(num_it), np.zeros_like(range(num_it)), 'k--')
    plt.xlabel("Time")
    plt.ylabel("h")
    plt.title("h")
    plt.legend(["1", "2", "3"])
# plt.ylim((-5, 5))  # highlight if violate safety


    plt.show()

plot_u()
