""" 

Simulator

author: Atsushi Sakai

"""

import numpy as np
import matplotlib.pyplot as plt
import math
import random

class Robot():
    def __init__(self):
        self.x = 25
        self.y = 0
    
    def visualize_robot(self):
        plt.plot(self.x, self.y, "*r")

    def move(self):
        self.y += 2

class Map():
    def __init__(self, src_path_map):
        self.map = np.genfromtxt(src_path_map)
        # self.width = width
        # self.height = height
        print("Finished reading map")

    def visualize_map(self):
        # fig = plt.figure()
        # plt.switch_backend('TkAgg')
        # mng = plt.get_current_fig_manager()
        # mng.resize(*mng.window.maxsize())
        # plt.ion()
        plt.imshow(self.map, cmap='Greys')
        # plt.axis([0, 800, 0, 800])
        # plt.draw()
        # plt.pause(1.0)


class LidarSimulator():

    def __init__(self):
        self.range_noise = 0.0

    def get_observation_points(self, vlist, angle_reso):
        x, y, angle, r = [], [], [], []

        # store all points
        for v in vlist:

            gx, gy = v.calc_global_contour()

            for vx, vy in zip(gx, gy):
                vangle = math.atan2(vy, vx)
                vr = np.hypot(vx, vy) * random.uniform(1.0 - self.range_noise,
                                                       1.0 + self.range_noise)

                x.append(vx)
                y.append(vy)
                angle.append(vangle)
                r.append(vr)

        # ray casting filter
        rx, ry = self.ray_casting_filter(x, y, angle, r, angle_reso)

        return rx, ry

    def ray_casting_filter(self, xl, yl, thetal, rangel, angle_reso):
        rx, ry = [], []
        rangedb = [float("inf") for _ in range(
            int(np.floor((np.pi * 2.0) / angle_reso)) + 1)]

        for i in range(len(thetal)):
            angleid = int(round(thetal[i] / angle_reso))

            if rangedb[angleid] > rangel[i]:
                rangedb[angleid] = rangel[i]

        for i in range(len(rangedb)):
            t = i * angle_reso
            if rangedb[i] != float("inf"):
                rx.append(rangedb[i] * np.cos(t))
                ry.append(rangedb[i] * np.sin(t))

        return rx, ry


def main():
    # print("start!!")

    # print("done!!")
    src_path_map = "map.dat"
    map1 = Map(src_path_map)
    map1.visualize_map()


if __name__ == '__main__':
    main()
