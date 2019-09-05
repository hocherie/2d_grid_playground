"""Given range measurements and current state, calculate safe attitude cmd.

Naive: take closest range. If unsafe, add an opposite vector to it."""
from simulator import Map, LidarSimulator, Robot
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import math
import random

SAFE_RANGE = 20

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
    print("closest", closest_range)
    closest_angle = angles[np.argmin(ranges)] # relative to body
    # TODO: confirm orientation

    # # 2. Get current body velocity #TODO can we do without velocity (may be noisy)
    # world_vel = state["xdot"]
    # # transform to body
    # R = get_rot_matrix(state["theta"])
    # body_vel = np.dot(R, world_vel)

    # 3. Calculate potential field gain based on closest distance
    apf_gain = np.clip(int((SAFE_RANGE - closest_range))/100, a_min=0, a_max=None) # active if negative #! hardcoded denominator

    # 4. Calculate roll and pitch for opposing vector 
    safe_pitch  = -apf_gain * closest_angle*np.cos(closest_angle)  #! DEBUG
    safe_roll   = apf_gain * closest_angle*np.sin(closest_angle)

    #! TODO: Hardcode safe yawrate as zero, and safe_thrust for general hover
    safe_yr = 0
    safe_thrust = 0.0040875

    attitude_cmd = [safe_roll, safe_pitch, safe_yr, safe_thrust]

    return attitude_cmd

def main():
    # Instantiate Map
    src_path_map = "data/two_obs.dat"
    map1 = Map(src_path_map)

    # Instantiate dense lidar for evaluation
    dense_lidar = LidarSimulator(map1, angles=np.arange(90)*4)

    # Instantiate Robot to be evaluated
    safe_robbie = Robot(map1, use_safe=True)

    for i in range(100):

        plt.cla()

        # Get safe cmd
        safe_atti_cmd = compute_safe_atti_cmd(safe_robbie.state, safe_robbie.lidar.ranges, safe_robbie.lidar.angles)
        [roll, pitch, yr, thrust] = safe_atti_cmd
        roll = np.clip(roll, np.radians(-30), np.radians(30))
        pitch = np.clip(pitch, np.radians(-30), np.radians(30))
        # Move robot
        safe_robbie.update([roll, pitch, 0])

        # Evaluation: Get distance to closest obstacle w/ dense lidar
        # safe_closest = distance_to_closest_obstacle(dense_lidar, safe_robbie)
        # safe_closest_list.append(safe_closest)
    
        # TODO: write to text file

        # Visualize
        map1.visualize_map()
        safe_robbie.visualize()
        plt.pause(0.1)





if __name__ == '__main__':
    main()
