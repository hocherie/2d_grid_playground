""" 

Adapted from Atsushi Sakai's Python Robotics
Simulator

"""

import numpy as np
import matplotlib.pyplot as plt
import math
import random
from bresenham import bresenham

MAX_RANGE = 1000
DISPSCALE = 5
SAFE_RANGE = 30

class Robot():
    def __init__(self, map1, lidar=None, pos_cont=None):
        self.x = 50
        self.y = 10
        self.hist_x = []
        self.hist_y = [] 
        self.map = map1

        # TODO: cleaner way?
        if lidar is None:
            self.lidar = LidarSimulator(map1)
        else:
            self.lidar = lidar

        # TODO: cleaner way?
        if pos_cont is None:
            self.pos_cont = PositionController(self.lidar)
        else:
            self.pos_cont = pos_cont
    
    def visualize_robot(self):
        plt.plot(self.x, self.y, "*r")
        plt.plot(self.hist_x, self.hist_y, ".")
    
    def visualize(self):
        """Visualizes robot and lidar"""
        self.visualize_robot()
        self.lidar.visualize_lidar((self.x, self.y))
        self.pos_cont.visualize_control((self.x, self.y))

    def move(self):
        self.hist_x.append(self.x)
        self.hist_y.append(self.y)
        self.x += self.pos_cont.u_x
        self.y += self.pos_cont.u_y
        # print(self.x, self.y)
        

    def update(self):
        """Moves robot and updates sensor readings"""

        self.lidar.update_reading((self.x, self.y))
        self.pos_cont.calc_control()
        self.move()
        
        


class Map():
    def __init__(self, src_path_map):
        self.map = np.genfromtxt(src_path_map)
        self.width = self.map.shape[1] #TODO: check
        self.height = self.map.shape[0]
        self.max_dist = math.sqrt(self.width**2 + self.height**2)
        print("Finished reading map of width " + 
            str(self.width) + "and height " + str(self.height))

    def visualize_map(self):
        plt.imshow(self.map, cmap='Greys')
        plt.axis([0, self.width, 0, self.height])


class PositionController():
    def __init__(self, lidar):
        self.u_x = 0
        self.u_y = 0
        self.og_control = None
        self.safe_control = None
        self.lidar = lidar

    def calc_control(self):
        og_control = self.calc_original_control()
        safe_control = self.calc_safe_control()
        self.u_x = self.og_control[0] + self.safe_control[0]
        self.u_y = self.og_control[1] + self.safe_control[1]

    # no account for safety
    # TODO: give better name
    def calc_original_control(self):
        og_ux = 0
        og_uy = 1
        self.og_control = (og_ux, og_uy)
        # return (og_ux, og_uy)

    def calc_safe_control(self):
        # Naive: choose minimum distance and push away. should have equilibrium point when at stopping limit
        # min_angle_ind = np.argmin(self.lidar.ranges)
        # self.lidar.reset_unsafe_range()
        # self.lidar.unsafe_range[min_angle_ind] = 1

        min_angle_ind = np.argmin(self.lidar.ranges)
        # print(min_angle_ind)
        # print(self.lidar.ranges)
        min_range = np.min(self.lidar.ranges)
        self.lidar.reset_unsafe_range()
        
        if min_range < SAFE_RANGE:
            self.lidar.reset_unsafe_range()
            self.lidar.unsafe_range[min_angle_ind] = 1

            # Push away
            unsafe_angle = self.lidar.angles[min_angle_ind]

            # TODO: cast to int
            safe_ux = int((SAFE_RANGE - min_range)//10 * np.cos(unsafe_angle + np.pi))
            safe_uy = int((SAFE_RANGE - min_range)//10 *
                          np.sin(unsafe_angle + np.pi))
            print("Executing safety maneuvers", safe_ux, safe_uy)
        
        else:
            safe_ux = 0
            safe_uy = 0

        self.safe_control = (safe_ux, safe_uy) #TODO



    def visualize_control(self, pos):
        # original control
        plt.plot([pos[0], pos[0]+self.og_control[0] * DISPSCALE],
                 [pos[1], pos[1]+self.og_control[1] * DISPSCALE], 'g')

        # safe control
        plt.plot([pos[0], pos[0]+self.safe_control[0] * DISPSCALE],
                 [pos[1], pos[1]+self.safe_control[1] * DISPSCALE], 'r')

        # final control
        plt.plot([pos[0], pos[0]+self.u_x * DISPSCALE],
                 [pos[1], pos[1]+self.u_y * DISPSCALE], 'b')
        # plt.plot(np.vstack((self.sensed_obs[:, 0], np.ones(len(self.angles)) * pos[0])),
        #          np.vstack((self.sensed_obs[:, 1], np.ones(len(self.angles)) * pos[1])), 'k', linewidth=0.1)


class LidarSimulator():
    def __init__(self, map1, angles=np.array(range(10)) * 33):
        self.range_noise = 0.0
        self.angles = angles * np.pi/180. # list in deg
        self.map = map1 #TODO: move to robot?
        self.sensed_obs = None 
        self.ranges = None
        self.unsafe_range = np.zeros_like(self.angles)

    def reset_unsafe_range(self):
        self.unsafe_range = np.zeros_like(self.angles)

    def get_bresenham_points(self, p1, p2):
        """Get list of coordinates of line (in tuples) from p1 and p2. 
        Input: points (tuple) ex. (x,y)"""

        return list(bresenham(p1[0], p1[1], p2[0], p2[1]))

    def update_reading(self, pos):
        """Update sensed obstacle locations and ranges."""
        closest_obs = [self.get_closest_obstacle(
            pos, angle) for angle in self.angles]
        self.sensed_obs = np.array(closest_obs)

        self.ranges = self.get_ranges(pos)

    def get_ranges(self, pos):
        """Get ranges given sensed obstacles"""

        ranges = []
        for obstacle in self.sensed_obs:
            if obstacle is None:
                ranges.append(10000)
            else:
                ranges.append(calc_dist((pos[0], pos[1]), obstacle))
        return np.array(ranges)

        
    def get_closest_obstacle(self, pos, angle):
        """"Get closest obs position given angle."""
        end_point_x = int(round(self.map.max_dist * np.cos(angle) + pos[0]))
        end_point_y = int(round(self.map.max_dist * np.sin(angle) + pos[1]))
        end_point = (end_point_x, end_point_y)
        along_line_pts = self.get_bresenham_points(pos, end_point)
        # make sure pts are within index
        along_line_pts = [pt for pt in along_line_pts if (pt[0] >= 0 and pt[0] < self.map.width)]
        along_line_pts = [pt for pt in along_line_pts if (
            pt[1] >= 0 and pt[1] < self.map.height)]
        along_line_pts = np.array(along_line_pts)
        # plt.plot(along_line_pts[:,0], along_line_pts[:,1], '.')
        along_line_occ = self.map.map[along_line_pts[:,1], along_line_pts[:,0]]
        # TODO: change to binary
        closest_obs_coord = along_line_pts[np.where(along_line_occ > 0.99)]
        if len(closest_obs_coord) == 0: # no obstacles
            # TODO: make into constant
            return [MAX_RANGE * np.cos(angle), MAX_RANGE * np.sin(angle)]
        else:
            return closest_obs_coord[0]
            

    def visualize_lidar(self, pos):
        # Plot hits
        plt.plot(self.sensed_obs[:, 0], self.sensed_obs[:, 1], "o")

        # Plot all rays
        plt.plot(np.vstack((self.sensed_obs[:, 0], np.ones(len(self.angles)) * pos[0])),
                 np.vstack((self.sensed_obs[:, 1], np.ones(len(self.angles)) * pos[1])), 'k', linewidth=0.1)

        # Plot unsafe range
        print(self.unsafe_range)
        unsafe_obs = self.sensed_obs[np.where(self.unsafe_range)]
        print(unsafe_obs)
        plt.plot(np.vstack((unsafe_obs[:,0], np.ones(len(unsafe_obs)) * pos[0])),
                 np.vstack((unsafe_obs[:, 1], np.ones(len(unsafe_obs)) * pos[1])), 'r', linewidth=0.5)


def calc_dist(p1, p2):
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)




def main():
    print("start!!")

    print("done!!")

if __name__ == '__main__':
    main()