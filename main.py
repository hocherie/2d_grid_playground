from simulator import Map, LidarSimulator, Robot
import numpy as np
import matplotlib.pyplot as plt
import math
import random


def main():
    print("start!!")


    
    # load map
    src_path_map = "data/blank_hallway.dat"
    map1 = Map(src_path_map)

    # initialize lidar
    lidar_angles = np.array(range(10)) * 30
    lidar = LidarSimulator(lidar_angles, map1)

    # initialize robot
    robbie = Robot(lidar)

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
