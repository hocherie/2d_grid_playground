import time 
from dynamics import QuadDynamics, init_state, QuadHistory, init_param_dict
from controller import *
from visualize_dynamics import *
import matplotlib.pyplot as plt
import numpy as np

class L1_control():
    def __init__(self):
        self.adapt_gain = 100 
        # TODO: find principled way to get cutoff frequency
        self.a_lp = 0.006  # Low Pass Filter freq~1Hz, 
        # https://www.wolframalpha.com/input/?i=arccos%28%28x%5E2%2B2x-2%29%2F%282x-2%29%29100%2F2pi+%3D+1
        self.velrate = np.zeros((3,))
        self.model_vel = np.zeros((3,))
        self.ksp = 50 # model gain
        self.true_disturbance = np.zeros((3,))
        self.l1_out = np.zeros((3,))

    def compute_control(self, current_vel, des_acc, dt):
        # Calculate current velocity error
        velerr = self.model_vel - current_vel 

        # Update model 
        velrate = self.true_disturbance - self.ksp * velerr
        # Add desired acceleration to velrate
        velrate += des_acc 
        
        disturbance_rate = -self.adapt_gain * velerr

        self.model_vel += dt *velrate 
        self.last_disturbance = np.copy(self.true_disturbance)
        self.true_disturbance += dt * disturbance_rate 
        
        # Apply low pass filter
        self.l1_out = self.exp_mov_avg()
        return self.l1_out
    
    def exp_mov_avg(self):
        # self.a_lp
        last_output = np.copy(self.l1_out)
        current_input = np.copy(self.true_disturbance)

        filtered_output = self.a_lp * \
            current_input + (1-self.a_lp) * last_output
        return filtered_output
    
def main():
    print("start")
    t_start = time.time()

    # Set desired position
    des_pos = np.array([0, 0, 0])

    # Initialize Robot State
    state = init_state()

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
    l1_control = L1_control()
    
    # Step through simulation
    sim_iter = 10000
    for t in range(sim_iter):
        des_pos = np.array([10,0,0])
        des_vel,_ = pi_position_control(state, des_pos)
        des_acc = pi_velocity_control(
            state, des_vel)
        des_acc = des_acc - l1_control.l1_out
        l1_control.compute_control(state["xdot"], des_acc, param_dict["dt"])
        u, des_theta_deg = go_to_acc(state, des_acc, param_dict)

        # Step dynamics and update state dict
        state = quad_dyn.step_dynamics(state, u)

        # update history for plotting
        quad_hist.update_history(state, des_theta_deg, des_vel, des_pos, param_dict["dt"],np.copy(l1_control.model_vel), np.copy(l1_control.l1_out))

    print("Time Elapsed:", time.time() - t_start)
    t = sim_iter - 1
    
    # Visualize quadrotor and angle error
    ax.cla()
    visualize_quad_quadhist(ax, quad_hist, t)
    visualize_error_quadhist(
        ax_x_error, ax_xd_error, ax_th_error, ax_thr_error, ax_xdd_error, quad_hist, t, param_dict["dt"])
    plt.show()

    


if __name__ == '__main__':
    main()
