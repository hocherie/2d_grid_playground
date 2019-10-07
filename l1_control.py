import time 
from dynamics import QuadDynamics, init_state, QuadHistory, init_param_dict
from controller import *
from visualize_dynamics import *
from min_snap_traj import *
import matplotlib.pyplot as plt
import numpy as np

class L1_control():
    def __init__(self):
        self.adapt_gain = 100 
        # TODO: find principled way to get cutoff frequency
        self.a_lp = 0.0006  # Low Pass Filter freq~1Hz, 
        # https://www.wolframalpha.com/input/?i=arccos%28%28x%5E2%2B2x-2%29%2F%282x-2%29%29100%2F2pi+%3D+1
        self.veld = np.zeros((3,)) # Change in velocity
        self.model_vel = np.zeros((3,))
        self.ksp = 50 # model gain
        self.disturbance = np.zeros((3,))
        self.l1_out = np.zeros((3,))

    def compute_control(self, current_vel, des_acc, dt):
        # Calculate current velocity error
        velerr = self.model_vel - current_vel 

        # Update model 
        veld = self.disturbance - self.ksp * velerr
        # Add desired acceleration to veld
        veld += des_acc 
        
        disturbance_rate = -self.adapt_gain * velerr

        self.model_vel += dt *veld 
        self.last_disturbance = np.copy(self.disturbance)
        self.disturbance += dt * disturbance_rate 
        
        # Apply low pass filter
        self.l1_out = self.exp_mov_avg()
        return self.l1_out
    
    def exp_mov_avg(self):
        # self.a_lp
        last_output = np.copy(self.l1_out)
        current_input = np.copy(self.disturbance)

        filtered_output = self.a_lp * \
            current_input + (1-self.a_lp) * last_output
        return filtered_output
    
def main():
    print("start")
    t_start = time.time()


    # Initialize Robot State
    state = init_state([0,0,0])

    # Initialize quadrotor history tracker
    quad_hist = QuadHistory()



    # Initialize quad dynamics
    param_dict = init_param_dict()
    quad_dyn = QuadDynamics(param_dict)
    Ts = quad_dyn.param_dict["dt"]
    l1_control = L1_control()
    # Step through simulation
    sim_iter = 10000
    # Initialize min snap
    trajSelect = np.array([6,4,0])
    ctrlType = "xyz_pos"
    trajHist = np.zeros((sim_iter, 19))
    stateHist = np.zeros((sim_iter, 3))
    traj = Trajectory(ctrlType, trajSelect)

    t = 0
    for i in range(sim_iter):
        sDes = traj.desiredState(t, Ts)
        trajHist[i, :] = sDes
        des_pos = sDes[0:3]
        # print(des_pos)
        des_vel = sDes[3:6]
        des_acc = sDes[6:9]

        # get current state
        current_pos = state["x"]
        current_vel = state["xdot"]
        # control quadrotor
        des_vel = pos_control(des_pos, current_pos, des_vel)
        des_acc = vel_control(des_vel, current_vel, des_acc)
        # des_acc += np.array([-1,0,0])
        des_acc = des_acc - l1_control.l1_out
        l1_control.compute_control(state["xdot"], des_acc, param_dict["dt"])
        u, des_theta_deg = go_to_acc(state, des_acc, param_dict)
        # Step dynamics and update state dict
        state = quad_dyn.step_dynamics(state, u)
        stateHist[i, :] = state["x"]

        # update history for plotting
        quad_hist.update_history(state, des_theta_deg, des_vel, des_pos, param_dict["dt"],np.copy(l1_control.model_vel), np.copy(l1_control.l1_out))
        t += Ts

    print("Time Elapsed:", time.time() - t_start)
    # t = sim_iter - 1
    
    # # Plot Velocity
    # plt.plot(np.arange(sim_iter) * Ts, trajHist[:, 3:6])

    # plt.legend(["x", "y", "z"])
    # np.save('trajvel', trajHist[:,3:6])
    # plt.show()

    # # Initialize visualization
    # fig = plt.figure()
    # ax = fig.add_subplot(2, 3, 1, projection='3d')
    # ax_x_error = fig.add_subplot(2, 3, 2)
    # ax_xd_error = fig.add_subplot(2, 3, 3)
    # ax_xdd_error = fig.add_subplot(2, 3, 4)
    # ax_th_error = fig.add_subplot(2, 3, 5)
    # ax_thr_error = fig.add_subplot(2, 3, 6)
    # # Visualize quadrotor and angle error
    # ax.cla()
    # visualize_quad_quadhist(ax, quad_hist, t)
    # visualize_error_quadhist(
    #     ax_x_error, ax_xd_error, ax_th_error, ax_thr_error, ax_xdd_error, quad_hist, t, param_dict["dt"])
    # plt.show()

    # Plot: compare t
    plt.style.use('fivethirtyeight')
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    for i in range(sim_iter):
        if (i % 50) == 0:
            print(i)
            plt.gca()
            ax.plot3D(trajHist[:i, 0], trajHist[:i, 1],
                        trajHist[:i, 2], 'r.', linewidth=0.01)
            ax.plot3D(stateHist[:i, 0], stateHist[:i, 1],
                        stateHist[:i, 2], 'b.', linewidth=0.01)
            # ax.scatter(stateHist[i, 0], stateHist[i, 1],
            #             stateHist[i, 2], c='k')
            plt.pause(0.0001)
            ax.set_xlim([-10, 10])
            ax.set_ylim([-10, 10])
            ax.set_zlim([-5, 5])
            ax.legend(["Desired", "Actual"])
    # # if is_plot:
    
    # ax.plot3D(trajHist[:, 0], trajHist[:, 1], trajHist[:, 2], 'r.')
    # ax.plot3D(stateHist[:, 0], stateHist[:, 1], stateHist[:, 2], 'b.')
    # ax.legend(["Desired", "Actual"])
    # plt.show()


    # # Plot: tracking error 
    # track_err = trajHist[:, :3] - stateHist
    # print(track_err)
    # np.save('l1_minsnap', track_err)
    # fig = plt.figure()
    # plt.plot(np.arange(sim_iter)*Ts, track_err)
    # plt.legend(["x", "y", "z"])
    # plt.title("Tracking Error")
    # plt.xlabel("Time")
    # plt.ylabel("Error (m)")
    # plt.show()

    


if __name__ == '__main__':
    main()
