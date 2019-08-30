"""dynamics.py
Simulate Simple Quadrotor Dynamics

`python dynamics.py` to see hovering drone
"""

import numpy as np
import numpy.matlib
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from visualize_dynamics import *
from sim_utils import *

# Physical constants
g = 9.81
m = 0.5
L = 0.25
k = 3e-6
b = 1e-7
I = np.diag([5e-3, 5e-3, 10e-3])
kd = 0.25
dt = 0.02

hist_theta = []
hist_des_theta = []
hist_thetadot = []
hist_x = []
hist_y = []
hist_z = []

def init_state():
    state = {"x": np.array([5, 0, 10]) , 
                "xdot": np.zeros(3,),
                # "theta": np.zeros(3,),
                # "thetadot": np.zeros(3,),
                "theta": np.radians(np.array([10, -7, 5])),  # ! hardcoded
                "thetadot": np.radians(np.array([10, 0, 0]))  # ! hardcoded
                }
    return state


def main():
    print("start")

    state = init_state()

    # Initialize visualization
    fig = plt.figure()
    ax = fig.add_subplot(1,3,1, projection='3d')
    ax_th_error = fig.add_subplot(1, 3, 2)
    ax_thr_error = fig.add_subplot(1,3,3)

    # ! hardcoded. should be given by velocity controller
    des_theta_deg = np.array([0, 0, 0])
    des_theta = np.radians(des_theta_deg)


    # Step through simulation
    for t in range(1000):
        ax.cla()

        u = pd_attitude_control(state, des_theta) # get control
        step_dynamics(state, u) # Step dynamcis and update state dict
        update_history(state, des_theta_deg) # update history for plotting

        # Visualize quadrotor and angle error
        visualize_quad(ax, state, hist_x, hist_y, hist_z)
        visualize_error(ax_th_error, ax_thr_error, hist_theta, hist_des_theta, hist_thetadot, dt)
def update_history(state, des_theta_deg_i):
    x = state["x"]
    hist_x.append(x[0])
    hist_y.append(x[1])
    hist_z.append(x[2])
    hist_theta.append(np.degrees(state["theta"]))
    hist_thetadot.append(np.degrees(state["thetadot"]))
    hist_des_theta.append(des_theta_deg_i)

def step_dynamics(state, u):
    """Step dynamics given current state and input. Update state dict."""
        # Compute angular velocity vector from angular velocities
    omega = thetadot2omega(state["thetadot"], state["theta"])

    # Compute linear and angular accelerations given input and state
    # TODO: combine state
    a = calc_acc(u, state["theta"], state["xdot"], m, g, k, kd)
    omegadot = calc_ang_acc(u, omega, I, L, b, k)

    # Compute next state
    omega = omega + dt * omegadot
    thetadot = omega2thetadot(omega, state["theta"])
    theta = state["theta"] + dt * state["thetadot"]
    xdot = state["xdot"] + dt * a
    x = state["x"] + dt * xdot

    # Update state dictionary
    state["x"] = x
    state["xdot"] = xdot
    state["theta"] = theta
    state["thetadot"] = thetadot


def compute_thrust(u,k):
    """Compute total thrust (in body frame) given control input and thrust coefficient. Used in calc_acc().

    thrust = k * sum(u)
    
    Parameters
    ----------
    u : (4, ) np.ndarray
        control input - (angular velocity)^squared of motors (rad^2/s^2)

    k : float
        thrust coefficient
    """
    T = np.array([0, 0, k*np.sum(u)])

    return T

def calc_torque(u, L, b, k):
    """Compute torque (body-frame), given control input, and coefficients. Used in calc_ang_acc()
    
    Parameters
    ----------
    u : (4, ) np.ndarray
        control input - (angular velocity)^squared of motors (rad^2/s^2)
    L : float
        distance from center of quadcopter to any propellers, to find torque (m).
    
    b : float # TODO: description
    
    k : float
        thrust coefficient

    Returns
    -------
    tau : (3,) np.ndarray
        torque in body frame (Nm)

    """
    tau = np.array([
            L * k * (u[0]-u[2]),
            L * k * (u[1]-u[3]),
            b * (u[0]-u[1] + u[2]-u[3])
    ])

    return tau

def calc_acc(u, theta, xdot, m, g, k, kd):
    """Computes linear acceleration (in inertial frame) given control input, gravity, thrust and drag.
    a = g + T_b+Fd/m

    Parameters
    ----------
    u : (4, ) np.ndarray
        control input - (angular velocity)^squared of motors (rad^2/s^2)
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
    a : (3, ) np.ndarray #TODO: confirm size
        linear acceleration in inertial frame (m/s^2)
    """
    gravity = np.array([0, 0, -g])
    R = get_rot_matrix(theta)
    thrust = compute_thrust(u, k)
    T = np.dot(R, thrust)
    Fd = -kd * xdot
    a = gravity + 1//m * T + Fd
    # a = gravity + (T+Fd)//m
    return a 

def calc_ang_acc(u, omega, I, L, b, k):
    """Computes angular acceleration (in body frame) given control input, angular velocity vector, inertial matrix.
    
    omegaddot = inv(I) * (torque - w x (Iw))

    Parameters
    ----------
    u : (4, ) np.ndarray
        control input - (angular velocity)^squared of motors (rad^2/s^2)
    omega : (3, ) np.ndarray 
        angular velcoity vector in body frame
    I : (3, 3) np.ndarray 
        inertia matrix
    L : float
        distance from center of quadcopter to any propellers, to find torque (m).
    b : float # TODO: description
    k : float
        thrust coefficient


    Returns
    -------
    omegaddot : (3, ) np.ndarray
        rotational acceleration in body frame #TODO: units
    """
    # Calculate torque given control input and physical constants
    tau = calc_torque(u, L, b, k)

    # Calculate body frame angular acceleration using Euler's equation
    omegaddot = np.dot(np.linalg.inv(
        I), (tau - np.cross(omega, np.dot(I, omega))))

    return omegaddot

def basic_input():
    """Return arbritrary input to test simulator"""
    return np.power(np.array([950, 700, 700, 700]),2)

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
    """Compute angular velocity vector from euler angle and associated rates.
    
    Uses Tait Bryan's z-y-x/yaw-pitch-roll. 

    Parameters
    ----------
    
    thetadot: (3, ) np.ndarray
        time derivative of euler angles (roll rate, pitch rate, yaw rate)

    theta: (3, ) np.ndarray
        euler angles in body frame (roll, pitch, yaw)

    Returns
    ---------
    w: (3, ) np.ndarray
        angular velocity vector (in body frame)
    
    """
    roll = theta[0]
    pitch = theta[1]
    yaw = theta[2]

    mult_matrix = np.array(
            [
            [1, 0, -np.sin(pitch)],
            [0, np.cos(roll), np.cos(pitch)*np.sin(roll)],
            [0, -np.sin(roll), np.cos(pitch)*np.cos(roll)]
            ]

    )

    w = np.dot(mult_matrix, thetadot)

    return w


def pd_attitude_control(state, des_theta):
    """Attitude controller (PD).
    
    Returns: u, motor speed
    
    """

    Kd = 4
    Kp = 7

    theta = state["theta"]
    thetadot = state["thetadot"]

    # Compute total thrust
    tot_thrust = (m * g) // (k * np.cos(theta[1]) * np.cos(theta[0]))

    # Compute errors
    # TODO: set thetadot to zero?
    e =  Kd * thetadot + Kp * (theta - des_theta)
    print("e_theta", e)

    # Compute control input (dynamic inversion)
    u = error2u(e, theta, tot_thrust, m, g, k, b, L, I)
    return u

def error2u(error, theta, tot_thrust, m, g, k, b, L, I):

    e0 = error[0]
    e1 = error[1]
    e2 = error[2]
    Ixx = I[0,0]
    Iyy = I[1,1]
    Izz = I[2,2]

    # rbase = (m*g) // (4*k*np.cos(theta[1])*np.cos(theta[0]))
    r0 = tot_thrust//4 - (2*b*e0*Ixx + e2*Izz*k*L)//(4*b*k*L)

    r1 = tot_thrust//4 + (e2*Izz)//(4*b) - (e1*Iyy)//(2*k*L)

    r2 = tot_thrust//4 + (2*b*e0*Ixx - e2*Izz*k*L)//(4*b*k*L)

    r3 = tot_thrust//4 + (e2*Izz)//(4*b) + (e1*Iyy)//(2*k*L)

    return np.array([r0, r1, r2, r3])



if __name__ == '__main__':
    main()
