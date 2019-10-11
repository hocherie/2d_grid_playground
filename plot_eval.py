import numpy as np
import matplotlib.pyplot as plt

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
