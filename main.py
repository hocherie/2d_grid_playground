from simulator import Map, LidarSimulator, Robot
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

    for i in range(100):
        plt.cla()
        print("Time " + str(i))
        robbie.move()
        map1.visualize_map()
        robbie.visualize_robot()
        # plt.draw()
        plt.pause(0.1)
        

    print("done!!")

if __name__ == '__main__':
    main()
