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
from dynamics import QuadDynamics, QuadHistory
from controller import go_to_position, pi_attitude_control
from visualize_dynamics import visualize_error_quadhist, visualize_quad_quadhist
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

    def dynamic_inversion(self, state, param_dict):
        """Invert dynamics. For outer loop, given v_tot, compute attitude.

        Parameters
        ----------
        self.v_tot
            total v: v_cr + v_lc - v_ad

        state #TODO: use self.x
            state

        Returns
        -------
        cmd
            command to actuator. For outer loop, [roll, pitch, yaw rate, thrust]

        """
        yaw = state["theta"][2]
        
        # specific_force = 
        self.v_tot = np.array([0,1,0])#! mock
        des_pitch = self.v_tot[0] * np.cos(yaw) + self.v_tot[1] * np.sin(yaw)
        des_roll = self.v_tot[0] * np.sin(yaw) - self.v_tot[1] * np.cos(yaw)

        # TODO: move to attitude controller?
        des_pitch = np.clip(des_pitch, np.radians(-30), np.radians(30))
        des_roll = np.clip(des_roll, np.radians(-30), np.radians(30))

        # TODO: currently, set yaw as constant
        des_yaw = yaw
        des_theta = [des_roll, des_pitch, des_yaw]
        


        # cmd = pi_attitude_control(state, des_theta, des_thrust_pc, param_dict)

        return des_theta

    def plant(self, des_theta, state, quad_dyn):
         # Thrust #TODO: assume hover thrust
        tot_u_constant = 408750 * 4  # hover, for four motors
        max_tot_u = 400000000.0
        thrust_pc_constant = tot_u_constant/max_tot_u
        des_thrust_pc = thrust_pc_constant

        u = pi_attitude_control(state, des_theta, des_thrust_pc, quad_dyn.param_dict)
        state = quad_dyn.step_dynamics(state, u)
        return state



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
    # Initialize MRAC controller
    mrac = MRAC_control()

    # Initialize quadrotor state #TODO: make to general function, not sure where
    state = {"x": np.array([5, 0, 10]),
                "xdot": np.zeros(3,),
                "theta": np.radians(np.array([0, 0, 30])),  # ! hardcoded
                "thetadot": np.radians(np.array([0, 0, 0]))  # ! hardcoded
                }
    # Initialize quadrotor dynamics and logger and parameters
    quad_dyn = QuadDynamics()
    quad_hist = QuadHistory()
    

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
        # u, des_theta, des_vel, des_pos = go_to_position(state, des_pos, quad_dyn.param_dict,
        #                integral_p_err=None, integral_v_err=None)
        des_theta = mrac.dynamic_inversion(state, quad_dyn.param_dict)
        state = mrac.plant(des_theta, state, quad_dyn)
        # Step dynamics and update state dict
        # state = quad_dyn.step_dynamics(state, u)
        # update history for plotting
        des_vel = [0,0,0] #! temp
        quad_hist.update_history(state, np.degrees(des_theta), des_vel, des_pos)

    print("plotting")
    for t in range(100):
        # # Visualize quadrotor and angle error
        ax.cla()
        visualize_quad_quadhist(ax, quad_hist, t)
        visualize_error_quadhist(
            ax_x_error, ax_xd_error, ax_th_error, ax_thr_error, quad_hist, t, quad_dyn.param_dict["dt"])

if __name__ == '__main__':
    main()
