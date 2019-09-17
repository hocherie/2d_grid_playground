"""mrac_control.py
Model Reference Adaptive Control

Author: Cherie Ho

Date: 9/16/2019

Based on Kannan and Johnson's `Model Reference Adaptive Control with a Constrained Linear Reference Model`

Includes:
* Reference Model
* Linear Compensator (PD)
* Adaptive Element
* Estimate Hedge
* Dynamic Inversion

"""
from dynamics import QuadDynamics
from controller import go_to_position
import numpy as np
import matplotlib.pyplot as plt

class MRAC_control:
    def __init__(self):
        self.x_c = None # external command. input to ref model
        self.x_r = None # states of reference model
        self.v_cr = None # output of reference model
        self.v_h = None # output of hedge
        self.v_ad = None # Output of adaptive element
        self.v_lc = None # Output of linear compensator
        self.v_tot = None
        self.model_track_err  = None  
        self.cmd = None # Final output to actuator
        self.x = None # current state

    def ref_model(self):
        """Linear reference model. Minimizes error
        between command and reference state.
        
        Allows one to impose prescribable limits on the evolution of the error states. Such as maximum speed and maximum acceleration.

        Parameters
        ----------
        self.x_r
            states of reference model
        self.x_c
            bounded external command signal
        self.v_h
            estimate hedge signal

        Updates
        -------
        self.v_cr
            for position control, acceleration from ref model
        """
        pass

    def linear_compensator(self):
        """Stabilizes linearized dynamics. PD control to minimize model tracking error.

        Parameters
        ----------
        self.model_track_err
            error between reference model and state
        
        Updates
        -------
        self.v_lc
            output from linear compensator

        """
        pass

    def dynamic_inversion(self):
        """Invert dynamics. For outer loop, given v_tot, compute attitude.

        Parameters
        ----------
        self.v_tot
            total v: v_cr + v_lc - v_ad

        Updates
        -------
        self.cmd
            command to actuator. For outer loop, [roll, pitch, yaw rate, thrust]

        """
        pass

    def adaptive_element(self):
        """
        Parameters
        ----------
        self.x
            robot state
        self.model_track_err
            model tracking error
        
        Update
        ------
        self.v_ad
            Adaptive element output

        """
        pass

    def update_model_track_err(self):
        "model_track_err = x_r - x"
        pass

    def compute_v_tot(self):
        "v_tot = v_cr + v_lc - v_ad"
        pass
        

def main():
    # Initialize quadrotor state #TODO: make to general function, not sure where
    state = {"x": np.array([5, 0, 10]),
                "xdot": np.zeros(3,),
                "theta": np.radians(np.array([0, 0, 30])),  # ! hardcoded
                "thetadot": np.radians(np.array([0, 0, 0]))  # ! hardcoded
                }
    # Initialize quadrotor dynamics
    quad_dyn = QuadDynamics()

    # Initialize visualization
    fig = plt.figure()
    ax = fig.add_subplot(2, 3, 1, projection='3d')
    ax_x_error = fig.add_subplot(2, 3, 2)
    ax_xd_error = fig.add_subplot(2, 3, 3)
    ax_th_error = fig.add_subplot(2, 3, 4)
    ax_thr_error = fig.add_subplot(2, 3, 5)

    # Initialize controller errors # TODO: move to objects
    integral_p_err = None
    integral_v_err = None

    # Set desired position
    des_pos = np.array([3, -3, 9])
    
    # Step through simulation
    for t in range(100):
        ax.cla()
        u = go_to_position(state, des_pos, quad_dyn.param_dict,
                       integral_p_err=None, integral_v_err=None)
        # des_vel, integral_p_err = pi_position_control(
        #     state, des_pos, integral_p_err)
        # des_thrust, des_theta, integral_v_err = pi_velocity_control(
        #     state, des_vel, integral_v_err)  # attitude control
        # des_theta_deg = np.degrees(des_theta)  # for logging
        # u = pi_attitude_control(
        #     state, des_theta, des_thrust, param_dict)  # attitude control
        # Step dynamcis and update state dict
        state = quad_dyn.step_dynamics(state, u)
        # update history for plotting
        update_history(state, des_theta_deg, des_vel, des_pos)
    print("plotting")
    for t in range(100):
        # # Visualize quadrotor and angle error
        ax.cla()
        visualize_quad(ax, hist_x[:t], hist_y[:t],
                       hist_z[:t], hist_pos[t], hist_theta[t])
        visualize_error(ax_x_error, ax_xd_error, ax_th_error, ax_thr_error,
                        hist_pos[:t+1], hist_xdot[:t+1], hist_theta[:t+1], hist_des_theta[:t+1], hist_thetadot[:t+1], dt, hist_des_xdot[:t+1], hist_des_x[:t+1])

if __name__ == '__main__':
    main()
