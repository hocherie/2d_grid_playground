
from simulator import Map, LidarSimulator, Robot
import numpy as np
import matplotlib.pyplot as plt
import math
import random
from safe_control import *


def head_to_wall():
    # Instantiate Map
    src_path_map = "data/blank_hallway.dat"
    map1 = Map(src_path_map)

    # Instantiate dense lidar for evaluation
    dense_lidar = LidarSimulator(map1, angles=np.arange(90)*4)

    # Instantiate Robot to be evaluated
    safe_robbie = Robot(map1, use_safe=True)
    safe_robbie.update([0, 0, 0]) # start with hover to get initial readings

    aggr_robbie = Robot(map1, use_safe=True)
    aggr_robbie.update([0,0,0])

    for i in range(200):

        plt.cla()

        # Get safe cmd
        safe_trigger, safe_atti_cmd = compute_safe_atti_cmd(safe_robbie.state, safe_robbie.lidar.ranges, safe_robbie.lidar.angles)
        print("velocity", safe_robbie.state["xdot"])
        if safe_trigger: # use safe att cmd
            [roll, pitch, yr, thrust] = safe_atti_cmd
            print("roll, pitch", np.degrees(roll), np.degrees(pitch))
            roll = np.clip(roll, np.radians(-30), np.radians(30))
            pitch = np.clip(pitch, np.radians(-30), np.radians(30))
            safe_robbie.update(safe_theta=[roll, pitch, 0], safe_kp=30)
        else:
            safe_robbie.update()

        safe_trigger, safe_atti_cmd = compute_safe_atti_cmd(aggr_robbie.state, aggr_robbie.lidar.ranges, aggr_robbie.lidar.angles)
        if safe_trigger: # use safe att cmd
            [roll, pitch, yr, thrust] = safe_atti_cmd
            print("roll, pitch", np.degrees(roll), np.degrees(pitch))
            roll = np.clip(roll, np.radians(-30), np.radians(30))
            pitch = np.clip(pitch, np.radians(-30), np.radians(30))
            aggr_robbie.update(safe_theta=[roll, pitch, 0], safe_kp=50)
        else:
            aggr_robbie.update()         


        

        # Evaluation: Get distance to closest obstacle w/ dense lidar
        # safe_closest = distance_to_closest_obstacle(dense_lidar, safe_robbie)
        # safe_closest_list.append(safe_closest)
    
        # TODO: write to text file

        # # Visualize
        # map1.visualize_map()
        # safe_robbie.visualize()
        # aggr_robbie.visualize()
        # plt.pause(0.1)
    
    plt.figure()
    plt.plot(safe_robbie.hist_y, label="Mild")
    plt.plot(aggr_robbie.hist_y, label="Aggressive")
    plt.legend()
    plt.show()

def main():
    print("start!!")

    # load map
    src_path_map = "data/two_obs.dat"
    map1 = Map(src_path_map)

    # lidar = LidarSimulator(map1)

    # initialize robot (initializes lidar with map) 
    robbie = Robot(map1, x=[60, 10, 10])

    for i in range(100):
        print("Time " + str(i))
        plt.cla()
        
        robbie.update()
        map1.visualize_map()
        robbie.visualize()
        plt.pause(0.1)
        
    print("done!!")

if __name__ == '__main__':
    main()
