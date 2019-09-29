import sys
sys.path.insert(1, '../Quadcopter_SimCon/Simulation')
from trajectory import Trajectory
from dynamics import QuadDynamics, init_state, QuadHistory, init_param_dict
from controller import go_to_acc

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

# TODO: make PID controller class 

def pos_control(des_pos, current_pos, des_vel):
    P_pos = 0 # TODO: check feedforward 
    #TODO: implement feedback
    return des_vel 

def vel_control(des_vel,current_vel, des_acc):
    P_vel = 0 # TODO: check feedforward
    # TODO: implement feedback
    return des_acc

def dynamic_inversion(des_acc, cur_yaw, param_dict):
    """Invert dynamics. For outer loop, given a_tot, compute attitude.
    """
    # yaw = state["theta"][2]
    # tot_u_constant = 408750 * 4  # hover, for four motors
    # specific_force = tot_u_constant  / param_dict["m"]

    # based on http://research.sabanciuniv.edu/33398/1/ICUAS2017_Final_ZAKI_UNEL_YILDIZ.pdf (Eq. 22-24)
    U1 = np.linalg.norm(des_acc - np.array([0, 0, param_dict["g"]]))
    des_pitch_noyaw = np.arcsin(des_acc[0] / U1)
    des_angle = [des_pitch_noyaw,
                 np.arcsin(des_acc[1] / (U1 * np.cos(des_pitch_noyaw)))]
    des_pitch = des_angle[0] * np.cos(yaw) + des_angle[1] * np.sin(yaw)
    des_roll = des_angle[0] * np.sin(yaw) - des_angle[1] * np.cos(yaw)

    # TODO: move to attitude controller?
    des_pitch = np.clip(des_pitch, np.radians(-30), np.radians(30))
    des_roll = np.clip(des_roll, np.radians(-30), np.radians(30))

    # TODO: currently, set yaw as constant
    des_yaw = yaw
    des_theta = [des_roll, des_pitch, des_yaw]

    # vertical (acc_z -> thrust)

    thrust = (param_dict["m"] * (des_acc[2] -
                                 param_dict["g"]))/param_dict["k"]  # T=ma/k
    max_tot_u = 400000000.0  # TODO: make in param_dict
    des_thrust_pc = thrust/max_tot_u

    return des_theta, des_thrust_pc

def main():
    # Initialize quad dynamics
    param_dict = init_param_dict()
    quad_dyn = QuadDynamics(param_dict)
    # l1_control = L1_control()
    Ts = quad_dyn.param_dict["dt"]
    state = init_state(np.array([0,0,0]))

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

        # print(sDes)
        t += Ts

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    for i in range(num_iter):
        if (i % 50) == 0:
            print(i)
            plt.gca()
            ax.plot3D(trajHist[:i,0], trajHist[:i,1],trajHist[:i,2],'r.')
            ax.plot3D(stateHist[:i,0], stateHist[:i,1],stateHist[:i,2],'b.')
            plt.pause(0.0001)
    # plt.show()

if __name__ == "__main__":
    # pass
    main()