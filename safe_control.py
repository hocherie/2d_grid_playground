"""Given range measurements and current state, calculate safe attitude cmd.

Naive: take closest range. If unsafe, add an opposite vector to it."""
from simulator import Map, LidarSimulator, Robot
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import math
import random

DANGER_RANGE = 30

def compute_safe_atti_cmd(state, ranges, angles):
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
    
    closest_angle = angles[np.argmin(ranges)] # relative to body
    print("closest", closest_range, closest_angle)

    # # 2. Get current body velocity #TODO can we do without velocity (may be noisy)
    # world_vel = state["xdot"]
    # # transform to body
    # R = get_rot_matrix(state["theta"])
    # body_vel = np.dot(R, world_vel)

    # 3. Calculate potential field gain based on closest distance
    
    # apf_function = (DANGER_RANGE - closest_range)/5
    # apf_gain = np.clip(apf_function, a_min=0, a_max=None) # active if negative #! hardcoded denominator

    apf_gain = np.radians(30)
    # 4. Calculate roll and pitch for opposing vector 
    safe_roll   = apf_gain  * np.sin(closest_angle)
    safe_pitch  = -apf_gain * np.cos(closest_angle) 
    

    #! TODO: Hardcode safe yawrate as zero, and safe_thrust for general hover
    safe_yr = 0
    safe_thrust = 0.0040875

    attitude_cmd = [safe_roll, safe_pitch, safe_yr, safe_thrust]

    safe_trigger =  closest_range < DANGER_RANGE # use control if unsafe

    return safe_trigger, attitude_cmd

def main():
   pass 




if __name__ == '__main__':
    main()
