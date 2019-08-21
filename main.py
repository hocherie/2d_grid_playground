from simulator import Map, LidarSim, Robot
import numpy as np
import matplotlib.pyplot as plt
import math
import random


def main():
    print("start!!")

    # initialize robot
    robbie = Robot()
    
    # load map
    src_path_map = "data/blank_hallway.dat"
    map1 = Map(src_path_map)

    # initialize lidar
    lidar_angles = np.array(range(10)) * 30
    lidar = LidarSim(lidar_angles, map1)

    for i in range(100):
        plt.cla()
        print("Time " + str(i))
        robbie.move()
        lidar.update_reading((robbie.x, robbie.y))
        # print(ranges)
        map1.visualize_map()
        robbie.visualize_robot()
        lidar.visualize_lidar((robbie.x, robbie.y))
        # plt.draw()
        plt.pause(0.1)
        
    print("done!!")

if __name__ == '__main__':
    main()
