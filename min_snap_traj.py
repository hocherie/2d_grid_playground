import sys
sys.path.insert(1, '../Quadcopter_SimCon/Simulation')
from trajectory import Trajectory
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def main():
    Ts = 0.005
    # Choose trajectory settings
    # --------------------------- 
    ctrlOptions = ["xyz_pos", "xy_vel_z_pos", "xyz_vel"]
    trajSelect = np.zeros(3)

    # Select Control Type             (0: xyz_pos,                  1: xy_vel_z_pos,            2: xyz_vel)
    ctrlType = ctrlOptions[0]   
    # Select Position Trajectory Type (0: hover,                    1: pos_waypoint_timed,      2: pos_waypoint_interp,    
    #                                  3: minimum velocity          4: minimum accel,           5: minimum jerk,           6: minimum snap
    #                                  7: minimum accel_stop        8: minimum jerk_stop        9: minimum snap_stop
    #                                 10: minimum jerk_full_stop   11: minimum snap_full_stop
    trajSelect[0] = 6          
    # Select Yaw Trajectory Type      (0: none                      1: yaw_waypoint_timed,      2: yaw_waypoint_interp     3: follow          4: zero)
    trajSelect[1] = 4           
    # Select if waypoint time is used, or if average speed is used to calculate waypoint time   (0: waypoint time,   1: average speed)
    trajSelect[2] = 0           
    print("Control type: {}".format(ctrlType))
    traj = Trajectory(ctrlType, trajSelect)
    # print(traj.coeff_x)
    num_iter = 10000
    trajHist = np.zeros((num_iter, 19))

    # Trajectory for First Desired States
    # ---------------------------
    t = 0
    for i in range(num_iter):
    # print("hi")
        sDes = traj.desiredState(t, Ts) 
        trajHist[i,:] = sDes
        # print(sDes)
        t += Ts

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.plot3D(trajHist[:,0], trajHist[:,1],trajHist[:,2],'.')
    plt.show()

if __name__ == "__main__":
    # pass
    main()