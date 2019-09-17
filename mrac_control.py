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

class MRAC_control():
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
        