from sim_utils import *
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

def visualize_quad(ax, state, hist_x, hist_y, hist_z):
    """Plot quadrotor 3D position and history"""
    x = state["x"]
    theta = state["theta"]
    R = get_rot_matrix(theta)
    plot_L = 1
    quad_ends_body = np.array(
        [[-plot_L, 0, 0], [plot_L, 0, 0], [0, -plot_L, 0], [0, plot_L, 0], [0, 0, 0], [0, 0, 0]]).T
    quad_ends_world = np.dot(R, quad_ends_body) + np.matlib.repmat(x, 6, 1).T
    # Plot Rods
    ax.plot3D(quad_ends_world[0, 0:2],
              quad_ends_world[1, 0:2], quad_ends_world[2, 0:2], 'b')
    ax.plot3D(quad_ends_world[0, 2:4],
              quad_ends_world[1, 2:4], quad_ends_world[2, 2:4], 'b')
    # Plot drone center
    ax.scatter3D(x[0], x[1], x[2], edgecolor="r", facecolor="r")

    # Plot history
    ax.scatter3D(hist_x, hist_y, hist_z, edgecolor="b",
                 facecolor="b", alpha=0.1)
    # ax_th_error.
    ax.set_xlim(x[0]-3, x[0]+3)
    ax.set_ylim(x[1]-3, x[1]+3)
    ax.set_zlim(x[2]-5, x[2]+5)
    plt.pause(0.0000001)


def visualize_error(ax_th_error, ax_thr_error, hist_theta, hist_des_theta, hist_thetadot, dt):
    # pass
    # ax.plot([0,1], [1,10],'b')

    # Angle Error
    ax_th_error.plot(np.array(range(len(hist_theta))) *
                     dt, np.array(hist_theta)[:, 0], 'k')
    ax_th_error.plot(np.array(range(len(hist_theta))) *
                     dt, np.array(hist_theta)[:, 1], 'b')
    ax_th_error.plot(np.array(range(len(hist_theta))) *
                     dt, np.array(hist_theta)[:, 2], 'r')
    # Desired angle
    ax_th_error.plot(np.array(range(len(hist_theta))) *
                     dt, np.array(hist_des_theta)[:, 0], 'k--')
    ax_th_error.plot(np.array(range(len(hist_theta))) *
                     dt, np.array(hist_des_theta)[:, 1], 'b--')
    ax_th_error.plot(np.array(range(len(hist_theta))) *
                     dt, np.array(hist_des_theta)[:, 2], 'r--')

    ax_th_error.legend(["Roll", "Pitch", "Yaw"])
    ax_th_error.set_ylim(-40, 40)

    # Angle Rate
    ax_thr_error.plot(np.array(range(len(hist_theta))) *
                      dt, np.array(hist_thetadot)[:, 0], 'k')
    ax_thr_error.plot(np.array(range(len(hist_theta))) *
                      dt, np.array(hist_thetadot)[:, 1], 'b')
    ax_thr_error.plot(np.array(range(len(hist_theta))) *
                      dt, np.array(hist_thetadot)[:, 2], 'r')
    # ax.plot(range(len(hist_theta)), np.array(des_theta)[:, 0])
    ax_thr_error.legend(["Roll Rate", "Pitch Rate", "Yaw Rate"])
    ax_thr_error.set_ylim(-40, 40)
    plt.pause(0.00001)
