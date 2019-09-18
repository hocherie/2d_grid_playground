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
        self.x_r = np.hstack((self.state["x"], self.state["xdot"])) # states of reference model #TODO: check
        self.v_cr = np.zeros((3,))  # output of reference model
        self.v_h = np.zeros((3,))  # output of hedge
        self.v_ad = np.zeros((3,))  # Output of adaptive element
        self.v_lc = np.zeros((3,)) # Output of linear compensator
        self.v_tot = np.zeros((3,))
        self.model_track_error = np.zeros((6,))
        self.cmd = None # Final output to actuator

        # Parameters
        self.lc_param = {"P": 1, "D": 1} #! handtuned
        self.Rp = np.diag(np.tile([self.lc_param["P"]], (3,))) #! hardcoded
        self.Rd = np.diag(np.tile([self.lc_param["D"]], (3,))) #! hardcoded
        self.rm_param = {"P": 1, "D": 1}  # ! handtuned

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
        model_error = self.x_c - self.x_r
        error_pos = model_error[0:3]  # command - reference
        des_vel = self.rm_param["P"] * error_pos
        self.v_cr = self.rm_param["D"] * (des_vel - self.x_r[3:])

    def update_model_state(self, dt):
        """Integrate reference states"""
        acc_ref = self.v_cr - self.v_h # acceleration, TODO: check if subtract hedge?
        self.x_r[3:] = self.x_r[3:] + acc_ref * dt  # vel_ref
        self.x_r[:3] = self.x_r[:3] + self.x_r[3:] * dt  # pos_ref
        # TODO: rotation?
        

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

        error_pos = self.model_track_error[0:3] # reference - state
        des_vel = self.lc_param["P"] * error_pos
        self.v_lc = self.lc_param["D"] * (des_vel - self.state["xdot"])
        
        

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
        # self.x_r = np.array([7,3,10, 0, 0, 0]) #! mock
        self.model_track_error = self.x_r - np.hstack((self.state["x"], self.state["xdot"]))
        print(self.model_track_error)

    def compute_v_tot(self):
        "v_tot = v_cr + v_lc - v_ad"

        self.v_tot = self.v_cr + self.v_lc - self.v_ad
        

def main():
    # Initialize quadrotor state #TODO: make to general function, not sure where
    state = {"x": np.array([5, 0, 10]),
             "xdot": np.zeros(3,),
             "theta": np.radians(np.array([0, 0, -25])),  
             "thetadot": np.radians(np.array([0, 0, 0]))  
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


    # Step through simulation
    for t in range(100):

        # Set desired position
        des_pos = np.array([5 + np.sin(0.1*t), 0.01*t, 10])
        mrac.x_c = np.hstack((des_pos, np.array([0, 0, 0])))

        # MRAC Loop
        mrac.update_model_track_err() # updates model_track_err
        mrac.ref_model() # updates self.v_cr
        mrac.linear_compensator() # updates v_lc

        mrac.compute_v_tot() # sums to v_tot
        des_theta = mrac.dynamic_inversion(quad_dyn.param_dict)

        mrac.update_model_state(quad_dyn.param_dict["dt"]) # integrate model acc 
        mrac.plant(des_theta, quad_dyn)


        # update history for plotting
        ax.cla()
        des_vel = [0,0,0] #! temp
        # update history for plotting
        quad_hist.update_history(mrac.state, np.degrees(
            des_theta), des_vel, des_pos, quad_dyn.param_dict["dt"])

    print("plotting")
    # for t in range(100):
    t = 99
    # # Visualize quadrotor and angle error
    ax.cla()
    visualize_quad_quadhist(ax, quad_hist, t)
    visualize_error_quadhist(
        ax_x_error, ax_xd_error, ax_th_error, ax_thr_error, ax_xdd_error, quad_hist, t, quad_dyn.param_dict["dt"])
    plt.show()

class AdaptNet():
    def __init__(self):
        self.n_in = 6
        self.n_hid = 10
        self.n_out = 3

        # Initialize weights
        self.V = np.zeros((self.n_in, self.n_hid))
        self.W = np.zeros((self.n_hid, self.n_out)) # TODO: what's good weight init

    def forward(self, X):
        # input to hidden
        z1 = self.V.T @ X # TODO: should it be transposed?
        z2 = self.sigmoid(z1)
        # TODO: include bias?

        # hidden to output
        out = self.W.T @ z2
        return out

    def compute_wgrad(self, X, Rp, Rd, track_error):
        """-gamma_w(sigma - sigma'V^TX)r^T + k abs(e) W"""
        # tracking error (pos_x, _y, _z, vel_x, _y, _z) pg.4
        # track_error = np.array([0,0,0,0,0,0]) #! mock
        # derivative of e = Ae + B[v_ad - disturbance]
        # A = np.array([[np.zeros((3,3)), np.eye(3)],
        #             [-Rp, -Rd]])
        B = np.array([np.zeros((3,3))], [np.eye(3)])


        # compute necessary components
        P1 = np.array([Rp.T @ Rp + 0.5 * Rp @ Rd.T @ Rd,    0.5 @ Rp @ Rd],
                        [0.5 @ Rp @ Rd,                   Rp])
        P = (1/(0.25*self.n_hid) + b_w.T @ b_w) # TODO: b_w
        r = (track_error.T @ P @ B).T # TODO: move to common

        # Final compute
        # TODO: is X hat different?
        sig_vt_x = self.sigmoid(self.V.T @ X)
        sigp_vt_x = self.sigmoid_prime(self.V.T @ X) # TODO: check sigmoid prime

        first_inner = (sig_vt_x - sigp_vt_x @ (self.V.T @ X)) @ self.r.T 

        second_inner = k * np.linalg.norm(track_error) * self.W
        w_grad = -self.gam_w @ (first_inner + second_inner)

        # TODO: add robustifying term
        return w_grad

    def compute_vgrad(self, X, Rp, Rd, track_error):
        """-gamma_v[x_in (r.T w.T sigma) + k * norm(track_error) * w]"""
        # TODO: is x_in different from state?
        # Compute necessary components
        P1 = np.zeros((6,6))
        P1[:3, :3] = Rp.T @ Rp + 0.5 * Rp @ Rd.T @ Rd # upper left
        P1[:3, 3:] = 0.5 * Rp @ Rd # upper right
        P1[3:, :3] = 0.5 * Rp @ Rd # lower left
        P1[3:, 3:] = Rp # lower right
        
        P = (1/(0.25*self.n_hid) + b_w.T @ b_w) * P1 # TODO: b_w
        r = (track_error.T @ P @ B).T # TODO: move to common

        # Final compute (eq. 52)
        first_inner = X @ (r.T @ self.W.T @ self.sigmoid_prime(self.V.T @ X))
        second_inner = k * np.linalg.norm(track_error) * self.V #TODO: @ or *
        v_grad = -self.gam_v * (first_inner + second_inner)

        return v_grad

    def updateWeights(self, X, Rp, Rd, track_error):
        v_grad = self.compute_vgrad(X, Rp, Rd, track_error)
        w_grad = self.compute_wgrad(X, Rp, Rd, track_error)

        self.W = self.W - self.l_w @ w_grad # TODO: check sign
        self.V = self.V - self.l_v @ v_grad
        
    
    def sigmoid(self, s):
        return 1/(1+np.exp(-s))

    def sigmoid_prime(self, s):
        return self.sigmoid(s) * (1-self.sigmoid(s))

def test_net():
    state = {"x": np.array([5, 0, 10]),
             "xdot": np.zeros(3,),
             "theta": np.radians(np.array([0, 0, -25])),  
             "thetadot": np.radians(np.array([0, 0, 0]))  
             }
    net = AdaptNet()
    mrac = MRAC_control(state)

    X = np.array([1,0,0,0,0,0]).T #! Mock
    ## NN iteration

    # Forward Pass (compute output with W and V)
    acc_ad_b = net.forward(X)
    print(acc_ad_b)

    # Back prop (update W, V)
    track_error = np.array([0.1, 0,0,0,0,0])
    net.updateWeights(X, mrac.Rp, mrac.Rd, track_error)


if __name__ == '__main__':
    test_net()

