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
    def __init__(self, state, goal=np.array([[10], [0]]), num_h=1):
        self.state = state
        self.shape_dict = {}
        Kp = 4
        Kd = 3
        self.K = np.array([Kp, Kd])
        self.goal=goal
        self.num_h = num_h
        self.use_safe = True
        # noise terms
        self.noise_x = np.zeros((3,))

    def add_state_noise(self, variance):
        # Apply random walk noise
        self.noise_x += (np.random.rand(3) - 0.5) * variance
        self.state["x"] += self.noise_x  # position
        # TODO: do for velocity too?

    def compute_h_hd(self):
        h = self.compute_h()
        hd = self.compute_hd()
        # print("h shape", h.shape)
        # print("hd shape", hd.shape)
        return np.hstack((h, hd)).astype(np.double)

    def compute_h(self, obs=None):
        # return self.compute_h_superellip(obs)
        return self.compute_h_box()

    def compute_hd(self, obs=None):
        # return self.compute_hd_superellip(obs)
        return self.compute_hd_box()

    def compute_A(self,obs=None):
        # return self.compute_A_superellip(obs)
        return self.compute_A_box()

    def compute_b(self,obs=None):
        # return self.compute_b_ellip(obs)
        return self.compute_b_box()

    def compute_safe_control(self,obs):
        if self.use_safe:
            A = self.compute_A(obs)
            assert(A.shape == (self.num_h,2))

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

        if np.linalg.norm(u_nom) > 1:
            u_nom = (u_nom/np.linalg.norm(u_nom))

        u_nom = np.array([0.02, 0.005])
        return u_nom.astype(np.double)

    # Box-specific functions
    def compute_h_box(self):
        hr = h_func(self.state["x"][0], self.state["x"][1], a, b, safety_dist)
        assert(hr.shape==(self.num_h,1))
        return hr
    
    def compute_hd_box(self):
        hd1 = -self.state["xdot"][1]
        hd2 = -self.state["xdot"][0]

        hd = np.vstack((hd1, hd2))
        assert(hd.shape==(self.num_h,1))
        return hd

    def compute_A_box(self):
        A = np.array([[0, -1], [-1, 0]])
        assert(A.shape==(self.num_h,2))
        # A2 = np.array()
        return A

    def compute_b_box(self):
        # print("K", self.K.shape)
        # print("hhd", self.compute_h_hd().shape)
        b_ineq = - self.compute_h_hd() @ np.atleast_2d(self.K).T
        # print(b_ineq.shape)
        assert(b_ineq.shape==(self.num_h,1))
        return b_ineq     

    # # Superellipsoid-specific functions
    # def compute_h_superellip(self, obs):
    #     rel_r = np.atleast_2d(self.state["x"][:2]).T - obs
    #     # TODO: a, safety_dist, obs, b
    #     hr = h_func(rel_r[0], rel_r[1], a, b, safety_dist)
    #     return hr

    # def compute_hd_superellip(self, obs):
    #     rel_r = np.atleast_2d(self.state["x"][:2]).T - obs
    #     rd = np.atleast_2d(self.state["xdot"][:2]).T
    #     term1 = (4 * np.power(rel_r[0],3) * rd[0])/(np.power(a,4))
    #     term2 = (4 * np.power(rel_r[1],3) * rd[1])/(np.power(b,4))
    #     return term1+term2

    # def compute_A_superellip(self, obs):
    #     rel_r = np.atleast_2d(self.state["x"][:2]).T - obs
    #     A0 = (4 * np.power(rel_r[0], 3))/(np.power(a, 4))
    #     A1 = (4 * np.power(rel_r[1], 3))/(np.power(b, 4))

    #     return np.array([np.hstack((A0, A1))])


    # def compute_b_ellip(self, obs):
    #     """extra + K * [h hd]"""
    #     rel_r = np.atleast_2d(self.state["x"][:2]).T - obs
    #     rd = np.array(np.array(self.state["xdot"])[:2])
    #     extra = -(
    #         (12 * np.square(rel_r[0]) * np.square(rd[0]))/np.power(a,4) +
    #         (12 * np.square(rel_r[1]) * np.square(rd[1]))/np.power(b, 4)
    #     )

    #     b_ineq = extra - self.K @ self.compute_h_hd(obs)
    #     return b_ineq



@np.vectorize
def h_func_superellip(r1, r2, a, b, safety_dist):
    hr = np.power(r1,4)/np.power(a, 4) + \
        np.power(r2, 4)/np.power(b, 4) - safety_dist
    return hr

# @np.vectorize
# def h_func_box1(r1, r2, a, b, safety_dist):
#     #! (positive y)
#     r_max = 5
#     hr1 = (r_max - r2) - safety_dist # bound on max y
#     return hr1

# @np.vectorize
# def h_func_box2(r1, r2, a, b, safety_dist):
#     #! (positive x)
#     r_max = 5
#     hr2 = (r_max - r1) - safety_dist # bound on max y
#     return hr2

def h_func_box(r1, r2, a, b, safety_dist):
    num_h = 2
    r_max = 5
    laser_angle = np.radians([0,90])
    if r1.shape == ():
        h = np.zeros((num_h, 1))
        
        for i in range(num_h):
            h_i = np.sin(laser_angle[i])*(r_max-r1) + np.cos(laser_angle[i]) * (r_max-r2) - safety_dist
            # print(np.cos(laser_angle) * (r_max-r2) )
            h[i] = h_i 
    else:
        h = np.empty((0,200))
        for i in range(num_h):
            h = np.vstack((h, np.sin(laser_angle[i])*(r_max-r1) + np.cos(laser_angle[i]) * (r_max-r2) - safety_dist))
    return h

def h_func(r1, r2, a, b, safety_dist):
    return h_func_box(r1, r2, a, b, safety_dist)

def plot_h(obs, num_h, h_func=h_func_box):

    plot_x = np.arange(-10, 10, 0.1)
    plot_y = np.arange(-10, 10, 0.1)
    xx, yy = np.meshgrid(plot_x, plot_y, sparse=True)
    z = h_func(xx, yy, a, b, safety_dist) 
    z = np.reshape(z, (2,200, 200))
    z = z > 0
    z = np.all(z,axis=0)
    h = plt.contourf(plot_x, plot_y, z, [-1, 0, 1], colors=["black","white"])
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.pause(0.00000001)

def run_trial(state, obs_loc,goal, num_it, variance):
    """ Run 1 trial"""
    # Initialize necessary classes
    dyn = QuadDynamics()
    num_h = 2
    ecbf = ECBF_control(state=state,goal=goal, num_h=num_h)
    state_hist = []
    new_obs = np.atleast_2d(obs_loc).T
    h_hist = np.zeros((num_it))
    
    noise_x = np.zeros(3) # keeps track of noise for random walk

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
        state_hist.append(state["x"]) # append true state
        if(tt % 100 == 0):
            print(tt)
            plt.cla()
            state_hist_plot = np.array(state_hist)
            nom_cont = ecbf.compute_nom_control()
            plt.plot([state_hist_plot[-1, 0], state_hist_plot[-1, 0] + 100 *
                      u_hat_acc[0]],
                     [state_hist_plot[-1, 1], state_hist_plot[-1, 1] + 100 * u_hat_acc[1]], label="Safe")
            plt.plot([state_hist_plot[-1, 0], state_hist_plot[-1, 0] + 100 *
                      nom_cont[0]],
                     [state_hist_plot[-1, 1], state_hist_plot[-1, 1] + 100 * nom_cont[1]],label="Nominal")
            plt.legend(["Safe", "Nominal"])
            plt.plot(state_hist_plot[:, 0], state_hist_plot[:, 1],'b')
            plt.plot(ecbf.goal[0], ecbf.goal[1], '*r')
            plt.plot(state_hist_plot[-1, 0], state_hist_plot[-1, 1], '*b') # current
            plt.xlim([-10, 10])
            plt.ylim([-10, 10])
            plot_h(new_obs, num_h)


    return np.array(state_hist), h_hist

def main():

    #! Experiment Variables
    num_it = 50000
    num_variance = 1
    num_trials = 1

    # Initialize result arrays
    state_hist_x_trials = np.zeros((num_it, num_variance))
    state_hist_y_trials = np.zeros((num_it, num_variance))
    # h_trials = np.zeros((num_it, num_variance))  # metric
    h_trial_mean_list = np.zeros((num_it, num_variance))
    h_trial_var_list = np.zeros((num_it, num_variance))


    for variance_i in range(num_variance):
        h_trial = np.zeros((num_it, num_trials))
        state_hist_x_trial = np.zeros((num_it, num_trials))
        state_hist_y_trial = np.zeros((num_it, num_trials))
        for trial in range(num_trials):
            #! Randomize trial variables. CHANGE!
            print("Trial: ",trial)
            # x_start_tr = np.random.rand()  # for randomizing start and goal
            # y_start_tr = np.random.rand() - 4
            # goal_x = np.random.rand() * 5 - 2.5
            # goal_y = np.random.rand() + 10
            x_start_tr = 0.5 #! Mock, test near obstacle
            y_start_tr = -4
            goal_x = 0.5
            goal_y = 10
            goal = np.array([[goal_x], [goal_y]])
            state = {"x": np.array([x_start_tr, y_start_tr, 10]),
                        "xdot": np.zeros(3,),
                        "theta": np.radians(np.array([0, 0, 0])), 
                        "thetadot": np.radians(np.array([0, 0, 0]))  
                        }
            obs_loc = [0,0]

            # ! use 0.0001 * trial num as variance for now
            state_hist, h_hist = run_trial(
                state, obs_loc, goal, num_it, variance=0.0001*variance_i)
            # Add trial results to list
            # state_hist_x_trial[:, trial] = state_hist[:, 0]
            # state_hist_y_trial[:, trial] = state_hist[:, 1]
            # h_trial[:,trial] = h_hist

        # # Calculate mean and variance for all trials
        # h_trial_mean = np.mean(h_trial, 1)
        # # print(h_trial_mean.shape)
        # # assert(h_trial_mean.shape == (num_it, 1))
        # h_trial_var = np.std(h_trial, 1)

        # Assign mean/variance trial to variance list
        
    #     h_trial_mean_list[:, variance_i] = np.copy(h_trial_mean)
    #     h_trial_var_list[:, variance_i] = np.copy(h_trial_var)

    # np.save("h_trial_mean_list", h_trial_mean_list)
    # np.save("h_trial_var_list", h_trial_var_list)





if __name__=="__main__":
    main()
