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
    def __init__(self, state):
        self.state = state
        self.x_c = None # external command. input to ref model
        self.x_r = None # states of reference model
        self.v_cr = np.zeros((3,))  # output of reference model
        self.v_h = np.zeros((3,))  # output of hedge
        self.v_ad = np.zeros((3,))  # Output of adaptive element
        self.v_lc = np.zeros((3,)) # Output of linear compensator
        self.v_tot = np.zeros((3,))
        self.model_track_err  = None  
        self.cmd = None # Final output to actuator

        # Parameters
        self.lc_param = {"P": 1, "D": 0}
        # self.x = None # current state

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

        Uses
        ----
        lc_param
            gains for linear compensator
        
        Returns
        -------
        v_lc
            output from linear compensator

        """
          # TODO: move to object init?
        self.model_track_error = np.array([1,0,0, 0,0,0]) #! mock. position x 3, velocity x 3

        error_pos = self.model_track_error[0:3] # reference - state
        error_vel = self.model_track_error[3:]

        print(error_pos)
        self.v_lc = self.lc_param["P"] * error_pos + self.lc_param["D"] * error_vel
        
        

    def dynamic_inversion(self, param_dict):
        """Invert dynamics. For outer loop, given v_tot, compute attitude.
        Similar to control allocator.

        TODO: do mapping?
        Parameters
        ----------
        self.v_tot
            total v: v_cr + v_lc - v_ad

        state #TODO: use self.x
            state

        Returns
        -------
        desired_theta: np.ndarray(3,)
            desired roll, pitch, yaw angle (rad) to attitude controller

        """
        yaw = self.state["theta"][2]
        # tot_u_constant = 408750 * 4  # hover, for four motors
        # specific_force = tot_u_constant  / param_dict["m"] 

        # based on http://research.sabanciuniv.edu/33398/1/ICUAS2017_Final_ZAKI_UNEL_YILDIZ.pdf (Eq. 22-24)
        U1 = np.linalg.norm(self.v_tot - np.array([0, 0, param_dict["g"]]))
        des_pitch_noyaw =  np.arcsin(self.v_tot[0] / U1)
        des_angle = [des_pitch_noyaw,
                     np.arcsin(self.v_tot[1] / (U1 * np.cos(des_pitch_noyaw)))]
        des_pitch = des_angle[0] * np.cos(yaw) + des_angle[1] * np.sin(yaw)
        des_roll = des_angle[0] * np.sin(yaw) - des_angle[1] * np.cos(yaw)

        # TODO: move to attitude controller?
        des_pitch = np.clip(des_pitch, np.radians(-30), np.radians(30))
        des_roll = np.clip(des_roll, np.radians(-30), np.radians(30))

        # TODO: currently, set yaw as constant
        des_yaw = yaw
        des_theta = [des_roll, des_pitch, des_yaw]
        
        return des_theta

    def plant(self, des_theta, quad_dyn):
         # Thrust #TODO: assume hover thrust
        tot_u_constant = 408750 * 4  # hover, for four motors
        max_tot_u = 400000000.0
        thrust_pc_constant = tot_u_constant/max_tot_u
        des_thrust_pc = thrust_pc_constant

        u = pi_attitude_control(self.state, des_theta, des_thrust_pc, quad_dyn.param_dict)
        self.state = quad_dyn.step_dynamics(self.state, u)
        # return state



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

        self.v_tot = self.v_cr + self.v_lc - self.v_ad
        

def main():
    # Initialize quadrotor state #TODO: make to general function, not sure where
    state = {"x": np.array([5, 0, 10]),
             "xdot": np.zeros(3,),
             "theta": np.radians(np.array([0, 0, 25])),  # ! hardcoded
             "thetadot": np.radians(np.array([0, 0, 0]))  # ! hardcoded
             }
             
    # Initialize MRAC controller
    mrac = MRAC_control(state)


    # Initialize quadrotor dynamics and logger and parameters
    quad_dyn = QuadDynamics()
    quad_hist = QuadHistory()
    

    # Initialize visualization
    fig = plt.figure()
    ax = fig.add_subplot(2, 3, 1, projection='3d')
    ax_x_error = fig.add_subplot(2, 3, 2)
    ax_xd_error = fig.add_subplot(2, 3, 3)
    ax_xdd_error = fig.add_subplot(2, 3, 4)
    ax_th_error = fig.add_subplot(2, 3, 5)
    ax_thr_error = fig.add_subplot(2, 3, 6)

    # Initialize controller errors # TODO: move to objects
    integral_p_err = None
    integral_v_err = None

    # Set desired position
    des_pos = np.array([3, -3, 9])
    
    # Step through simulation
    for t in range(100):

        # MRAC Loop
        mrac.linear_compensator() # updates v_lc
        mrac.compute_v_tot() # sums to v_tot
        des_theta = mrac.dynamic_inversion(quad_dyn.param_dict)
        mrac.plant(des_theta, quad_dyn)


        # update history for plotting
        ax.cla()
        des_vel = [0,0,0] #! temp
        # update history for plotting
        quad_hist.update_history(mrac.state, np.degrees(
            des_theta), des_vel, des_pos, quad_dyn.param_dict["dt"])

    print("plotting")
    for t in range(100):
        # # Visualize quadrotor and angle error
        ax.cla()
        visualize_quad_quadhist(ax, quad_hist, t)
        visualize_error_quadhist(
            ax_x_error, ax_xd_error, ax_th_error, ax_thr_error, ax_xdd_error, quad_hist, t, quad_dyn.param_dict["dt"])

if __name__ == '__main__':
    main()
