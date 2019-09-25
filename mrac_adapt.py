"""mrac_adapt.py
Model Reference Adaptive Control's adaptive element.
Single hidden-layer neural network.
Run with `mrac_adapt.py`.

Author: Cherie Ho

Date: 9/24/2019

Based on Kannan and Johnson's `Model Reference Adaptive Control with a Constrained Linear Reference Model`

"""
import numpy as np
class MRAC_Adapt():
    def __init__(self):
        self.n_in = 6
        self.n_hid = 10
        self.n_out = 3

        # Initialize weights (W,V)
        # TODO: what's good weight init
        # plus one to include theta row
        self.V = np.zeros((self.n_in + 1, self.n_hid))
        # plus one to include theta row
        self.W = np.zeros((self.n_hid + 1, self.n_out))

        # Initialize bias constants
        self.bw = 1  # ! taken from Basti's code
        self.bv = 1  # ! taken from Basti's code

        # Initialize bias weights (theta_w, theta_v)
        self.theta_v = np.zeros((self.n_hid, 1))
        self.theta_w = np.zeros((self.n_out, 1))

        # Initialize learning parameters
        self.k_track = 0.01  # taken from Basti's code
        self.gam_v = 0.1  # taken from Basti's
        self.gam_w = 0.1  # taken from Basti's
        self.k = 0.0001  # taken from Basti's

    def forward(self, X):
        X_bar = np.vstack((self.bw, X))
        # input to hidden
        # 10x 1 # TODO: should it be transposed?
        z1 = self.V.T @ X_bar + self.bv * self.theta_v
        assert(z1.shape == (10, 1))
        z2 = self.sigmoid_bw(z1)  # 11 x 1
        assert(z2.shape == (11, 1))
        # TODO: include bias?

        # hidden to output
        out = self.W.T @ z2 + self.bw * self.theta_w  # 3 x 1
        assert(out.shape == (3, 1))
        return out

    def compute_wgrad(self, X_in, Rp, Rd, track_error):
        """-gamma_w(sigma - sigma'V^TX)r^T + k abs(e) W"""
        # tracking error (pos_x, _y, _z, vel_x, _y, _z) pg.4
        # derivative of e = Ae + B[v_ad - disturbance]
        # A = np.array([[np.zeros((3,3)), np.eye(3)],
        #             [-Rp, -Rd]])
        B = np.vstack((np.zeros((3, 3)), np.eye(3)))
        X_bar = np.vstack((self.bv, X_in))  # TODO: make common

        # Compute necessary components
        P1 = np.zeros((6, 6))
        P1[:3, :3] = Rp.T @ Rp + 0.5 * Rp @ Rd.T @ Rd  # upper left
        P1[:3, 3:] = 0.5 * Rp @ Rd  # upper right
        P1[3:, :3] = 0.5 * Rp @ Rd  # lower left
        P1[3:, 3:] = Rp  # lower right
        P = (1/(0.25*self.n_hid) + np.power(self.bw, 2)) * P1  # TODO: b_w
        r = (track_error.T @ P @ B).T  # TODO: move to common

        # Final compute
        # TODO: is X hat different?
        sig_vt_x = self.sigmoid_bw(self.V.T @ X_bar)
        # TODO: check sigmoid prime
        sigp_vt_x = self.sigmoid_p_bw(self.V.T @ X_bar)
        assert(sigp_vt_x.shape == (11, 10))
        first_inner = (sig_vt_x - sigp_vt_x @ (self.V.T @ X_bar)) @ r.T

        second_inner = self.k * np.linalg.norm(track_error) * self.W
        w_grad = -self.gam_w * (first_inner + second_inner)

        # TODO: add robustifying term
        return w_grad

    def compute_vgrad(self, X_in, Rp, Rd, track_error):
        """-gamma_v[x_in (r.T w.T sigma) + k * norm(track_error) * w]"""
        # augment X_in
        X_bar = np.vstack((self.bv, X_in))  # TODO: make common

        # TODO: is x_in different from state?
        # Compute necessary components
        B = np.vstack((np.zeros((3, 3)), np.eye(3)))
        P1 = np.zeros((6, 6))
        P1[:3, :3] = Rp.T @ Rp + 0.5 * Rp @ Rd.T @ Rd  # upper left
        P1[:3, 3:] = 0.5 * Rp @ Rd  # upper right
        P1[3:, :3] = 0.5 * Rp @ Rd  # lower left
        P1[3:, 3:] = Rp  # lower right

        P = (1/(0.25*self.n_hid) + np.power(self.bw, 2)) * P1  # TODO: b_w
        r = (track_error.T @ P @ B).T  # TODO: move to common

        # Final compute (eq. 52)
        first_inner = X_bar @ (
            r.T @ self.W.T @ self.sigmoid_p_bw(self.V.T @ X_bar))
        second_inner = self.k_track * \
            np.linalg.norm(track_error) * self.V  # TODO: @ or *
        v_grad = -self.gam_v * (first_inner + second_inner)

        return v_grad

    def updateWeights(self, X_in, Rp, Rd, track_error):
        v_grad = self.compute_vgrad(X_in, Rp, Rd, track_error)
        w_grad = self.compute_wgrad(X_in, Rp, Rd, track_error)

        self.W = self.W + self.gam_w * w_grad  # TODO: check sign
        self.V = self.V + self.gam_v * v_grad  # TODO: is gamma learning rate?
        # print("weight updated")

    def sigmoid_orig(self, s):
        return 1/(1+np.exp(-s))

    def sigmoid_prime(self, s):
        return np.multiply(self.sigmoid_orig(s),  (1-self.sigmoid_orig(s)))

    def sigmoid_bw(self, s):
        """Add bw as first element"""
        return np.vstack((self.bw, self.sigmoid_orig(s)))

    def sigmoid_p_bw(self, s):
        """Add bw as first element"""

        # TODO: check
        return np.vstack((np.zeros((1, self.n_hid)), np.diag(np.ndarray.flatten(self.sigmoid_prime(s)))))




if __name__ == '__main__':
    pass
    # test_net()
    # main()
