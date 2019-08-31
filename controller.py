import numpy as np
import math 

def pi_position_control(state, des_pos, integral_p_err=None):
    if integral_p_err is None:
        integral_p_err = np.zeros((3,))

    Px = -0.7
    Ix = 0  # -0.005
    Py = -0.7
    Iy = 0  # 0.005
    Pz = 1

    [x, y, z] = state["x"]
    [x_d, y_d, z_d] = des_pos
    yaw = state["theta"][2]

    # Compute error
    p_err = state["x"] - des_pos
    # accumulate error integral
    integral_p_err += p_err

    # Get PID Error
    # TODO: vectorize

    pid_err_x = Px * p_err[0] + Ix * integral_p_err[0]
    pid_err_y = Py * p_err[1] + Iy * integral_p_err[1]
    pid_err_z = Pz * p_err[2]  # TODO: project onto attitude angle?

    # TODO: implement for z vel
    des_xv = pid_err_x # * np.cos(yaw) + pid_err_y * np.sin(yaw)
    des_yv = pid_err_y #* np.sin(yaw) - pid_err_y * np.cos(yaw)

    # TODO: currently, set z as constant
    des_zv = 0

    return np.array([des_xv, des_yv, des_zv]), integral_p_err

def pi_velocity_control(state, des_vel, integral_v_err=None):
    """
    Assume desire zero angular velocity?

    #TODO: only for x,y velocity

    Parameter
    ---------
    state : dict 
        contains current x, xdot, theta, thetadot
        
    des_vel : (3, ) np.ndarray
        desired linear velocity

    integral_v_err : (3, ) np.ndarray
        keeps track of integral error

    Returns
    -------
    uv : (3, ) np.ndarray
        roll, pitch, yaw 
    """
    if integral_v_err is None:
        integral_v_err = np.zeros((3,))
    
    Pxd = -0.12
    Ixd = -0.005 #-0.005
    Pyd = -0.12
    Iyd = -0.005 #0.005
    Pzd = 1
    # TODO: change to return roll pitch yawrate thrust

    [xv, yv, zv] = state["xdot"]
    [xv_d, yv_d, zv_d] = des_vel
    yaw = state["theta"][2]

    # Compute error
    v_err = state["xdot"] - des_vel
    # accumulate error integral
    integral_v_err += v_err
    
    # Get PID Error
    # TODO: vectorize
    
    pid_err_x = Pxd * v_err[0] + Ixd * integral_v_err[0]
    pid_err_y = Pyd * v_err[1] + Iyd * integral_v_err[1]
    pid_err_z = Pzd * v_err[2] # TODO: project onto attitude angle?
    

    # TODO: implement for z vel
    des_pitch = pid_err_x * np.cos(yaw) + pid_err_y * np.sin(yaw)
    des_roll = pid_err_x * np.sin(yaw) - pid_err_y * np.cos(yaw)

    # TODO: currently, set yaw as constant
    des_yaw = state["theta"][2]
    print("pid_error", pid_err_x, pid_err_y)
    print("des pitch, roll", des_pitch, des_roll)

    return np.array([des_roll, des_pitch, state["theta"][2]]), integral_v_err


def pi_attitude_control(state, des_theta,param_dict):
    """Attitude controller (PD). Uses current theta and theta dot.
    
    Parameter
    ---------
    state : dict 
        contains current x, xdot, theta, thetadot

    k : float
        thrust coefficient

    Returns
    -------
    u : (4, ) np.ndarray
        control input - (angular velocity)^squared of motors (rad^2/s^2)
    
    """

    Kd = 10
    Kp = 30

    # TODO: make into class, have param_dict as class member
    g = param_dict["g"]
    m = param_dict["m"]
    L = param_dict["L"]
    k = param_dict["k"]
    b = param_dict["b"]
    I = param_dict["I"]
    kd = param_dict["kd"]
    dt = param_dict["dt"]

    theta = state["theta"]
    thetadot = state["thetadot"]

    # Compute total thrust
    tot_thrust = (m * g) // (k * np.cos(theta[1]) * np.cos(theta[0]))

    # Compute errors
    # TODO: set thetadot to zero?
    e = Kd * thetadot + Kp * np.array(map(wrap2pi, theta - des_theta))
    # print("e_theta", e)

    # Compute control input given angular error (dynamic inversion)
    u = angerr2u(e, theta, tot_thrust, param_dict)
    return u

def wrap2pi(ang_diff):
    """For angle difference."""
    while ang_diff > np.pi//2 or ang_diff < -np.pi//2:
        print(ang_diff)
        if ang_diff > np.pi//2:
            ang_diff -= np.pi
        else: # < -np.pi//2
            ang_diff += np.pi
    
    return ang_diff
def angerr2u(error, theta, tot_thrust, param_dict):
    """Compute control input given angular error. Closed form specification
    with dynamics inversion.

    Parameters
    ----------
    error
    """
    # TODO: make into class, have param_dict as class member
    g = param_dict["g"]
    m = param_dict["m"]
    L = param_dict["L"]
    k = param_dict["k"]
    b = param_dict["b"]
    I = param_dict["I"]
    kd = param_dict["kd"]
    dt = param_dict["dt"]
    
    e0 = error[0]
    e1 = error[1]
    e2 = error[2]
    Ixx = I[0, 0]
    Iyy = I[1, 1]
    Izz = I[2, 2]

    r0 = tot_thrust//4 - (2*b*e0*Ixx + e2*Izz*k*L)//(4*b*k*L)

    r1 = tot_thrust//4 + (e2*Izz)//(4*b) - (e1*Iyy)//(2*k*L)

    r2 = tot_thrust//4 + (2*b*e0*Ixx - e2*Izz*k*L)//(4*b*k*L)

    r3 = tot_thrust//4 + (e2*Izz)//(4*b) + (e1*Iyy)//(2*k*L)

    return np.array([r0, r1, r2, r3])
