from dynamics import QuadDynamics, QuadHistory
from controller import go_to_position, pi_attitude_control
from visualize_dynamics import visualize_error_quadhist, visualize_quad_quadhist
from mrac_adapt import MRAC_Adapt
from sim_utils import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm


adapt_data = np.loadtxt("yes_adapt.txt")
print(adapt_data)
no_adapt_data = np.loadtxt("no_adapt.txt")

colors = cm.rainbow(np.linspace(0, 1, 3))
num_data = len(adapt_data[:, 0])
dt = 0.1
plot_t = np.arange(num_data)*dt
plt.plot(plot_t, adapt_data[:,0], 'k')

# plt.plot(adapt_data[:,1], 'b')
# plt.plot(adapt_data[:, 2], 'g')
# plt.scatter([1, 2, 3], [4, 5, 6], color=)

# plt.plot(no_adapt_data, "--")
no_adapt_width = 0.5
plt.plot(plot_t,
         no_adapt_data[:, 0], 'k--', linewidth=no_adapt_width)
plt.plot(plot_t, np.zeros_like(plot_t), 'k', linewidth=no_adapt_width)
plt.legend(["x (adapt)",  "x (no adapt)"])
plt.title("Tracking Error")
# plt.plot(no_adapt_data[:, 1], 'b--', linewidth=no_adapt_width)
# plt.plot(no_adapt_data[:, 2], 'g--', linewidth=no_adapt_width)
# plt.legend(["x (adapt)", "y (adapt)", "z (adapt)", "x", "y", "z"])
plt.ylim([-1, 1])
plt.ylabel("Tracking Error (m)")
plt.xlabel("Time (s)")
# plt.title("Tracking Error (No Adaptive)")
plt.show()
