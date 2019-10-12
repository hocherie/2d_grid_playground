from dynamics import QuadDynamics
from controller import *
import numpy as np
import matplotlib.pyplot as plt
from cvxopt import matrix
from cvxopt import solvers

a = 1
b = 1
safety_dist = 0.25 # TODO: change

class ECBF_control():
    def __init__(self, state, goal=np.array([[0], [10]])):
        self.state = state
        self.shape_dict = {}
        Kp = 1
        Kd = 10
        self.K = np.array([Kp, Kd])
        self.goal=goal
        self.use_safe = True
        # noise terms
        self.noise_x = np.zeros((3,))

    def add_state_noise(self, variance):
        # Apply random walk noise
        self.noise_x += (np.random.rand(3) - 0.5) * variance
        self.state["x"] += self.noise_x  # position
        # TODO: do for velocity too?

    def compute_h(self, obs=np.array([[0], [0]]).T, state_x=None):
        if state_x is None:
            rel_r = np.atleast_2d(self.state["x"][:2]).T - obs
        else:
            rel_r = np.atleast_2d(state_x[:2]).T - obs
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
            np.random.seed()
            optimized_u += np.random.random()*np.linalg.norm(optimized_u)*.1

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

    plot_x = np.arange(-5, 5, 0.01)
    plot_y = np.arange(-5, 5, 0.01)
    xx, yy = np.meshgrid(plot_x, plot_y, sparse=True)
    z = h_func(xx - obs[0], yy - obs[1], a, b, safety_dist) > 0
    h = plt.contourf(plot_x, plot_y, z, [-1, 0, 1], colors=["black","white"])
    plt.xlabel("X")
    plt.ylabel("Y")
    # plt.pause(0.00000001)

def run_trial(start_state, obs_loc,goal, num_it, variance):
    """ Run 1 trial"""
    # Initialize necessary classes
    dyn = QuadDynamics()
    ecbf = ECBF_control(state=start_state,goal=goal)
    state_hist = []
    new_obs = np.atleast_2d(obs_loc).T
    h_hist = np.zeros((num_it))

    random_walk_noise = np.zeros(3) # keeps track of noise for random walk
    noisy_state = start_state  # start noisy state as given state
    true_state = start_state  # start true state as given state

    # Loop through iterations
    for tt in range(num_it):
        # accumulate gaussian noise to make random walk noise (zero-mean)
        random_walk_noise += np.random.normal(loc=0, scale=variance)

        # get safe control acc (use noisy state)
        u_hat_acc = ecbf.compute_safe_control(obs=new_obs) 
        u_hat_acc = np.ndarray.flatten(
            np.array(np.vstack((u_hat_acc, np.zeros((1, 1))))))  # add zero acceleration to z
        assert(u_hat_acc.shape == (3,))

        # Get final motor speed for dynamics (use noisy state)
        u_motor = go_to_acceleration(
            noisy_state, u_hat_acc, dyn.param_dict)  # desired motor rate ^2
        
        # Get new true state with u_motor (use true state as dynamics is not based on fake data)
        true_state = dyn.step_dynamics(true_state, u_motor)
        
        # Update noisy state
        noisy_state["x"] = true_state["x"] + random_walk_noise
        assert(noisy_state["x"].shape == (3,))
        ecbf.state = noisy_state # use noisy state for ecbf control

        # Append true state and true h for plotting
        state_hist.append(true_state["x"])  # append true state
        h_hist[tt] = ecbf.compute_h(new_obs, state_x=true_state["x"])

    return np.array(state_hist), h_hist

def main():

    #! Experiment Variables
    num_it = 2000
    num_variance = 1
    num_trials = 1

    # Initialize result arrays
    state_hist_x_trials = np.zeros((num_it, num_variance))
    state_hist_y_trials = np.zeros((num_it, num_variance))
    # h_trials = np.zeros((num_it, num_variance))  # metric
    h_trial_mean_list = np.zeros((num_it, num_variance))
    h_trial_var_list = np.zeros((num_it, num_variance))


    # for variance_i in range(num_variance):
    h_trial = np.zeros((num_it, num_trials))
    state_hist_x_trial = np.zeros((num_it, num_trials))
    state_hist_y_trial = np.zeros((num_it, num_trials))
    r_start_list = np.zeros((2,num_trials))
    r_end_list = np.zeros((2, num_trials))
    for trial in range(num_trials):
        #! Randomize trial variables. CHANGE!
        print("Trial: ",trial)
        start_x_list = [1.5, 1.5, -1, -2, -4, 2.6]
        start_y_list = [4, -3.8, -3, 4.7, -2, 0.8]
        goal_x_list = [-2, 1.5, 0, 0.6, 2.7, -3.9]
        goal_y_list = [-4, 4.5, 4.5, -3.9, 0.5, -0.6]
        # x_start_tr = np.random.rand()*4 - 2  # for randomizing start and goal
        # y_start_tr = np.random.rand() - 4
        # goal_x = np.random.rand() * 4 - 2
        # goal_y = np.random.rand() + 4
        x_start_tr = start_x_list[trial]
        y_start_tr = start_y_list[trial]
        goal_x = goal_x_list[trial]
        goal_y = goal_y_list[trial]
        # x_start_tr = 0.5 #! Mock, test near obstacle
        # y_start_tr = -4
        # goal_x = 0.5
        # goal_y = 10
        goal = np.array([[goal_x], [goal_y]])
        state = {"x": np.array([x_start_tr, y_start_tr, 10]),
                    "xdot": np.zeros(3,),
                    "theta": np.radians(np.array([0, 0, 0])), 
                    "thetadot": np.radians(np.array([0, 0, 0]))  
                    }
        obs_loc = [0,0]

        #! hardcoded variance
        state_hist, h_hist = run_trial(
            state, obs_loc, goal, num_it, variance=0.00003)
        # Add trial results to list
        state_hist_x_trial[:, trial] = state_hist[:, 0]
        state_hist_y_trial[:, trial] = state_hist[:, 1]
        h_trial[:, trial] = h_hist

    np.save("state_hist_x_trial_noise", state_hist_x_trial)
    np.save("state_hist_y_trial_noise", state_hist_y_trial)
    # Plot robot trajectories
    plt.figure()
    
    # plt.plot(state_hist_x_trial, state_hist_y_trial)
    plot_h(np.atleast_2d(obs_loc).T)  # plot safe set
    plt.scatter(state_hist_x_trial, state_hist_y_trial, c=h_trial < 0, s=1)
    # print(state_hist_x_trial.shape)

    

    # plt.figure()
    # plt.scatter(range(len(h_trial)), h_trial, c=h_trial<0)
    # plt.plot(np.zeros_like(h_trial),'k--')
    plt.show()

    






if __name__=="__main__":
    main()
