
from simulator import Map, LidarSimulator, Robot
import numpy as np
import matplotlib.pyplot as plt
import math
import random
from safe_control import *


def main():
    # Instantiate Map
    src_path_map = "data/two_obs.dat"
    map1 = Map(src_path_map)

    # Instantiate dense lidar for evaluation
    dense_lidar = LidarSimulator(map1, angles=np.arange(90)*4)

    # Instantiate Robot to be evaluated
    safe_robbie = Robot(map1, use_safe=True)
    safe_robbie.update([0, 0, 0]) # start with hover to get initial readings

    for i in range(100):

        plt.cla()

        # Get safe cmd
        safe_atti_cmd = compute_safe_atti_cmd(safe_robbie.state, safe_robbie.lidar.ranges, safe_robbie.lidar.angles)
        [roll, pitch, yr, thrust] = safe_atti_cmd
        print("roll, pitch", roll, pitch)
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

# def main():
#     print("start!!")

#     # load map
#     src_path_map = "data/two_obs.dat"
#     map1 = Map(src_path_map)

#     # lidar = LidarSimulator(map1)

#     # initialize robot (initializes lidar with map) 
#     robbie = Robot(map1)

#     for i in range(100):
#         print("Time " + str(i))
#         plt.cla()
        
#         robbie.update()
#         map1.visualize_map()
#         robbie.visualize()
#         plt.pause(0.1)
        
#     print("done!!")

if __name__ == '__main__':
    main()
