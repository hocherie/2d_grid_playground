"""Given range measurements and current state, calculate safe attitude cmd.

Naive: take closest range. If unsafe, add an opposite vector to it."""
import numpy as np

def safe_control(ranges, angles):
    """
    Parameters
    ----------
    range
    state (unused for now)

    Returns
    -------
    att : (4, ) np.ndarray
        [roll, pitch, yaw rate, thrust]
    """
    # 1. Get closest range measurement and angle
    closest_range = np.min(ranges)
    closest_angle = angles[np.argmin(self.lidar.ranges)] # relative to body
    # TODO: confirm orientation

    # 2. Get current body velocity #TODO can we do without velocity (may be noisy)
    world_vel = state["xdot"]
    # transform to body
    R = get_rot_matrix(state["theta"])
    body_vel = np.dot(R, world_vel)

    # 3. Calculate roll and pitch for opposing vector 
    safe_roll  = - apf_gain * closest_angle*np.cos(closest_angle) 
    safe_pitch = - apf_gain * closest_angle*np.sin(closest_angle)



    des_attitude = [safe_roll, safe_pitch, safe_yr, safe_thrust]