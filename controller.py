import numpy as np

# def pd_velocity_control():
def pd_attitude_control(state, des_theta,param_dict):
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

    Kd = 4
    Kp = 7

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
    e = Kd * thetadot + Kp * (theta - des_theta)
    # print("e_theta", e)

    # Compute control input given angular error (dynamic inversion)
    u = angerr2u(e, theta, tot_thrust, param_dict)
    return u


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
