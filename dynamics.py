"""dynamics.py
Simulate Simple Quadrotor Dynamics

`python dynamics.py` to see hovering drone
"""

import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt



def main():
    print("start")


    # Physical constants
    g = 9.81
    m = 0.5
    L = 0.25
    k = 3e-6
    b = 1e-7
    I = np.diag([5e-3, 5e-3, 10e-3])
    kd = 0.25
    dt = 0.01

    # Initialize
    x = np.array([0,0,10])
    xdot = np.zeros_like(x)
    theta = np.zeros_like(x)
    thetadot = np.zeros_like(x)

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    hist_x = []
    hist_y = []
    hist_z = []
    # Step through simulation
    for t in range(100):
        plt.cla()
        # Get control input
        u = calc_control()

        # Compute angular velocity vector from angular velocities
        omega = thetadot2omega(thetadot, theta)


        # Compute angular and linear accelerations given input and state
        # TODO: combine state
        a = calc_acc(u, theta, xdot, m, g, k, kd)
        omegadot = calc_ang_acc(u, omega, I, L, b, k)

        # Compute velocity and state
        omega = omega + dt * omegadot
        thetadot = omega2thetadot(omega, theta)
        theta = theta + dt * thetadot
        xdot = xdot + dt * a
        x = x + dt * xdot
        print(x)

        visualize_quad(ax, x, hist_x, hist_y, hist_z)

        hist_x.append(x[0])
        hist_y.append(x[1])
        hist_z.append(x[2])


def visualize_quad(ax, x, hist_x, hist_y, hist_z):
    """Plot quadrotor 3D position and history"""
    ax.scatter3D(x[0], x[1], x[2], edgecolor="r", facecolor="r")
    ax.scatter3D(hist_x, hist_y, hist_z, edgecolor="b", facecolor="b", alpha=0.1)
    ax.set_zlim(0, 20)
    plt.pause(0.1)

def compute_thrust(u,k):
    """compute thrust from input and thrust coefficient"""
    T = np.array([0, 0, k*np.sum(u)])

    return T

def calc_torque(u, L, b, k):
    """Compute torque, given input, and coefficients"""
    tau = np.array([
            L * k * (u[0]-u[2]),
            L * k * (u[1]-u[3]),
            b * (u[0]-u[1]+u[2]-u[3])
    ])

    return tau

def calc_acc(u, theta, xdot, m, g, k, kd):
    """Computes linear acceleration (in inertial frame) given control input, gravity, thrust and drag.
    a = g + T_b+Fd/m

    Parameters
    ----------
    u : (4, ) np.ndarray
        control input #TODO: which unit
    theta : (3, ) np.ndarray 
        rpy angle in body frame (radian) 
    xdot : (3, ) np.ndarray
        linear velocity in body frame (m/s), for drag calc 
    m : float
        mass of quadrotor (kg)
    g : float
        gravitational acceleration (m/s^2)
    k : float
        thrust coefficient
    kd : float
        drag coefficient

    Returns
    -------
    a : float
        linear acceleration in inertial frame (m/s^2)
    """
    gravity = np.array([0, 0, -g])
    R = get_rot_matrix(theta)
    thrust = compute_thrust(u, k)
    T = np.dot(R, thrust)
    Fd = -kd * xdot
    a = gravity + 1//m * (T + Fd)
    return a 

def calc_ang_acc(u, omega, I, L, b, k):

    tau = calc_torque(u, L, b, k)
    omegaddot = np.dot(np.linalg.inv(
        I), (tau - np.cross(omega, np.dot(I, omega))))
    return omegaddot

def calc_control():
    return np.array([10, 10, 10, 10])*100000

def omega2thetadot(omega, theta):
    mult_matrix = np.array(
        [
            [1, 0, -np.sin(theta[1])],
            [0, np.cos(theta[0]), np.cos(theta[1])*np.sin(theta[0])],
            [0, -np.sin(theta[0]), np.cos(theta[1])*np.cos(theta[0])]
        ]

    , dtype='float')

    mult_inv = np.linalg.inv(mult_matrix)
    thetadot = np.dot(mult_inv, omega)

    return thetadot

def thetadot2omega(thetadot, theta):
    mult_matrix = np.array(
            [
            [1, 0, -np.sin(theta[1])],
            [0, np.cos(theta[0]), np.cos(theta[1])*np.sin(theta[0])],
            [0, -np.sin(theta[0]), np.cos(theta[1])*np.cos(theta[0])]
            ]

    )

    w = np.dot(mult_matrix, thetadot)

    return w

def get_rot_matrix(angles):
    [phi, theta, psi] = angles
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    cthe = np.cos(theta)
    sthe = np.sin(theta)
    cpsi = np.cos(psi)
    spsi = np.sin(psi)

    rot_mat = np.array([[cthe * cpsi, sphi * sthe * cpsi - cphi * spsi, cphi * sthe * cpsi + sphi * spsi],
                        [cthe * spsi, sphi * sthe * spsi + cphi *
                            cpsi, cphi * sthe * spsi - sphi * cpsi],
                        [-sthe,       cthe * sphi,                      cthe * cphi]])
    return rot_mat

# def pd_control(state, thetadot):
#         Kd = 4
#         Kp = 3

#         # Compute total thrust
#         tot_thrust = 

if __name__ == '__main__':
    main()
