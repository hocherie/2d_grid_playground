from dynamics import QuadDynamics
from controller import *
import numpy as np
import matplotlib.pyplot as plt
from cvxopt import matrix
from cvxopt import solvers

a = 1
b = 1
safety_dist = 1 # TODO: change

class ECBF_control():
    def __init__(self, state, goal=np.array([[0], [10]])):
        self.state = state
        self.shape_dict = {}
        Kp = 3
        Kd = 4
        self.K = np.array([Kp, Kd])
        self.goal=goal
        self.use_safe = True

    def compute_h(self, obs=np.array([[0], [0]]).T):
        rel_r = np.atleast_2d(self.state["x"][:2]).T - obs
        # TODO: a, safety_dist, obs, b
        hr = h_func(rel_r[0], rel_r[1], a, b, safety_dist)
        return hr

    def compute_hd(self, obs):
        rel_r = np.atleast_2d(self.state["x"][:2]).T - obs
        rd = np.atleast_2d(self.state["xdot"][:2]).T
        term1 = (4 * np.power(rel_r[0],3) * rd[0])/(np.power(a,4))
        term2 = (4 * np.power(rel_r[1],3) * rd[1])/(np.power(b,4))
        return term1+term2

    def compute_A(self, obs):
        rel_r = np.atleast_2d(self.state["x"][:2]).T - obs
        A0 = (4 * np.power(rel_r[0], 3))/(np.power(a, 4))
        A1 = (4 * np.power(rel_r[1], 3))/(np.power(b, 4))

        return np.array([np.hstack((A0, A1))])

    def compute_h_hd(self, obs):
        h = self.compute_h(obs)
        hd = self.compute_hd(obs)

        return np.vstack((h, hd)).astype(np.double)

    def compute_b(self, obs):
        """extra + K * [h hd]"""
        rel_r = np.atleast_2d(self.state["x"][:2]).T - obs
        rd = np.array(np.array(self.state["xdot"])[:2])
        extra = -(
            (12 * np.square(rel_r[0]) * np.square(rd[0]))/np.power(a,4) +
            (12 * np.square(rel_r[1]) * np.square(rd[1]))/np.power(b, 4)
        )

        b_ineq = extra - self.K @ self.compute_h_hd(obs)
        return b_ineq

    def compute_safe_control(self,obs):
        if self.use_safe:
            A = self.compute_A(obs)
            assert(A.shape == (1,2))

            b_ineq = self.compute_b(obs)

            #Make CVXOPT quadratic programming problem
            P = matrix(np.eye(2), tc='d')
            q = -1 * matrix(self.compute_nom_control(), tc='d')
            G = -1 * matrix(A.astype(np.double), tc='d')

            h = -1 * matrix(b_ineq.astype(np.double), tc='d')
            solvers.options['show_progress'] = False
            sol = solvers.qp(P,q,G, h, verbose=False) # get dictionary for solution

            optimized_u = sol['x']

        else:
            optimized_u = self.compute_nom_control()


        return optimized_u

    def compute_nom_control(self, Kn=np.array([-0.08, -0.2])):
        #! mock
        vd = Kn[0]*(np.atleast_2d(self.state["x"][:2]).T - self.goal)
        u_nom = Kn[1]*(np.atleast_2d(self.state["xdot"][:2]).T - vd)

        if np.linalg.norm(u_nom) > 0.01:
            u_nom = (u_nom/np.linalg.norm(u_nom))* 0.01
        return u_nom.astype(np.double)

@np.vectorize
def h_func(r1, r2, a, b, safety_dist):
    hr = np.power(r1,4)/np.power(a, 4) + \
        np.power(r2, 4)/np.power(b, 4) - safety_dist
    return hr

def plot_h(obs):

    plot_x = np.arange(-10, 10, 0.1)
    plot_y = np.arange(-10, 10, 0.1)
    xx, yy = np.meshgrid(plot_x, plot_y, sparse=True)
    z = h_func(xx - obs[0], yy - obs[1], a, b, safety_dist) > 0
    h = plt.contourf(plot_x, plot_y, z, [-1, 0, 1], colors=["black","white"])
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.pause(0.00000001)

def run_trial(state, obs_loc,goal, num_it):
    """ Run 1 trial"""
    # Initialize necessary classes
    dyn = QuadDynamics()
    ecbf = ECBF_control(state=state,goal=goal)
    state_hist = []
    new_obs = np.atleast_2d(obs_loc).T
    h_hist = np.zeros((num_it))

    # Loop through iterations
    for tt in range(num_it):
        u_hat_acc = ecbf.compute_safe_control(obs=new_obs)
        u_hat_acc = np.ndarray.flatten(
            np.array(np.vstack((u_hat_acc, np.zeros((1, 1))))))  # acceleration
        assert(u_hat_acc.shape == (3,))
        u_motor = go_to_acceleration(
            state, u_hat_acc, dyn.param_dict)  # desired motor rate ^2
        state = dyn.step_dynamics(state, u_motor)
        ecbf.state = state
        state_hist.append(state["x"])
        h_hist[tt] = ecbf.compute_h(new_obs)

        # if(tt % 500 == 0):
        #     print(tt)
    return np.array(state_hist), h_hist

def main():

    #! Experiment Variables
    num_it = 5000
    num_trials = 10

    # Initialize result arrays
    state_hist_x_trials = np.zeros((num_it, num_trials))
    state_hist_y_trials = np.zeros((num_it, num_trials))
    h_trials = np.zeros((num_it, num_trials)) # metric

    for trial in range(num_trials):
        #! Randomize trial variables. CHANGE!
        print("Trial: ",trial)
        x_start_tr = np.random.rand() 
        y_start_tr = np.random.rand() - 4
        goal_x = np.random.rand() * 5 - 2.5
        goal_y = np.random.rand() + 10
        goal = np.array([[goal_x], [goal_y]])
        state = {"x": np.array([x_start_tr, y_start_tr, 10]),
                    "xdot": np.zeros(3,),
                    "theta": np.radians(np.array([0, 0, 0])), 
                    "thetadot": np.radians(np.array([0, 0, 0]))  
                    }
        obs_loc = [0,0]


        state_hist, h_hist = run_trial(state, obs_loc, goal, num_it)
        # Add trial results to list
        state_hist_x_trials[:, trial] = state_hist[:, 0]
        state_hist_y_trials[:, trial] = state_hist[:, 1]
        h_trials[:,trial] = h_hist

    # Plot metrics over time
    plt.plot(h_trials)
    plt.xlabel("Time")
    plt.ylabel("h")
    plt.title("h")

    # Plot vehicle trajectories
    plt.figure()
    plt.plot(state_hist_x_trials, state_hist_y_trials)
    plot_h(np.atleast_2d(obs_loc).T)
    plt.show()




if __name__=="__main__":
    main()
