import sys
sys.path.insert(1, '../Quadcopter_SimCon-master/Simulation')
from trajectory import Trajectory
from dynamics import QuadDynamics, init_state, QuadHistory, init_param_dict
from controller import go_to_acc
# from l1_control import L1_control

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

# TODO: make PID controller class 

def pos_control(des_pos, current_pos, des_vel):
    P_pos = -1 
    # P_pos = 0
    fb_pos = P_pos * (current_pos - des_pos) 
    corrected_vel = des_vel + fb_pos # add feedforward 
    return corrected_vel 

def vel_control(des_vel,current_vel, des_acc):
    P_vel = -5
    # P_vel = 0 
    fb_vel = P_vel * (current_vel - des_vel)
    corrected_acc = des_acc + fb_vel
    return corrected_acc


def main():
    # Initialize quadrotor history tracker
    quad_hist = QuadHistory()

    # Initialize visualization
    fig = plt.figure()
    ax = fig.add_subplot(2, 3, 1, projection='3d')
    ax_x_error = fig.add_subplot(2, 3, 2)
    ax_xd_error = fig.add_subplot(2, 3, 3)
    ax_xdd_error = fig.add_subplot(2, 3, 4)
    ax_th_error = fig.add_subplot(2, 3, 5)
    ax_thr_error = fig.add_subplot(2, 3, 6)

    # Initialize quad dynamics
    param_dict = init_param_dict()
    quad_dyn = QuadDynamics(param_dict)
    # l1_control = L1_control()
    Ts = quad_dyn.param_dict["dt"]
    state = init_state(np.array([0,0,0]))
    # l1_control = L1_control()

    # TODO: set different waypoints
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
    stateHist = np.zeros((num_iter, 3))

    # Trajectory for First Desired States
    # ---------------------------
    t = 0
    for i in range(num_iter):
    # print("hi")
        # get desired state from minimum snap traj
        sDes = traj.desiredState(t, Ts) 
        # sDes = np.zeros((19,))
        # print(sDes)
        trajHist[i,:] = sDes
        des_pos = sDes[0:3]
        des_vel = sDes[3:6]
        des_acc = sDes[6:9]

        # get current state 
        current_pos = state["x"]
        current_vel = state["xdot"]



        # control quadrotor 
        des_vel = pos_control(des_pos, current_pos, des_vel)
        des_acc = vel_control(des_vel, current_vel, des_acc)
        u, des_theta_deg = go_to_acc(state, des_acc, param_dict)

        # Step dynamics and update state dict
        state = quad_dyn.step_dynamics(state, u)
        stateHist[i,:] = state["x"]

        # update history for plotting
        quad_hist.update_history(state, des_theta_deg, des_vel, des_pos, param_dict["dt"], np.copy(
            np.zeros((3,))), np.copy(np.zeros((3,))))
        # print(sDes)
        t += Ts

    # Visualization
    is_plot = True 
    is_animate = False  

    # Plot Velocity
    plt.plot(np.arange(num_iter) * Ts, trajHist[:,3:6])
    plt.legend(["x", "y", "z"])
    plt.show()

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    if is_plot:
        ax.plot3D(trajHist[:,0], trajHist[:,1],trajHist[:,2],'r.')
        ax.plot3D(stateHist[:,0], stateHist[:,1],stateHist[:,2],'b.')
        plt.show()
        # plt.pause(0.0001)
    if is_animate:
        for i in range(num_iter):
            if (i % 50) == 0:
                print(i)
                plt.gca()
                ax.plot3D(trajHist[:i,0], trajHist[:i,1],trajHist[:i,2],'r.',linewidth=0.25)
                ax.plot3D(stateHist[:i,0], stateHist[:i,1],stateHist[:i,2],'b.',linewidth=0.25)
                ax.scatter(stateHist[i, 0], stateHist[i, 1],
                          stateHist[i, 2], c='k')
                plt.pause(0.0001)
                ax.set_xlim([-10,10])
                ax.set_ylim([-10,10])
                ax.set_zlim([-5,5])
    plt.show()

if __name__ == "__main__":
    # pass
    main()
