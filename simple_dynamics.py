import numpy as np
import matplotlib.pyplot as plt
from cvxopt import matrix
from cvxopt import solvers

a = 1
b = 1
safety_dist = 1
class SimpleDynamics():
    def __init__(self):
        ## State space
        r = np.array([np.array([3,-4])]).T # position
        rd = np.array([np.array([0, 0])]).T  # velocity
        self.state = {"r":r, "rd":rd}
        ## Params
        self.dt = 10e-3

        # self.u = zeros(2,1) # acceleration, control input
    
    def step(self, u):
        # rdd = self.u
        rd = self.state["rd"] + self.dt * u
        r = self.state["r"] + self.dt * self.state["rd"]

        self.state["rd"] = rd
        self.state["r"]  = r

class ECBF_control():
    def __init__(self, state, goal=np.array([[0], [4]])):
        self.state = state
        self.shape_dict = {} #TODO: a, b
        # self.gain_dict = {} #TODO: Kp, Kd
        Kp = 0
        Kd = 0
        self.K = np.array([Kp, Kd])
        self.goal=goal
        # pass

    def plot_h(self):
        obs = np.array([0,0]) #! mock
        
        plot_x = np.arange(-5, 5, 0.1)
        plot_y = np.arange(-5, 5, 0.1)
        xx, yy = np.meshgrid(plot_x, plot_y, sparse=True)
        z = h_func(xx,yy, a, b, safety_dist) > 0
        h = plt.contourf(plot_x, plot_y, z, [-1, 0, 1])
        # h = plt.contourf(plot_x, plot_y, z)
        plt.xlabel("X")
        plt.ylabel("Y")
        proxy = [plt.Rectangle((0, 0), 1, 1, fc=pc.get_facecolor()[0])
                 for pc in h.collections]
        plt.legend(proxy, ["Unsafe: range(-1 to 0)","Safe: range(0 to 1)"])

        
        plt.show()



    def compute_h(self, obs=np.array([[0], [0]]).T):
        rel_r = self.state["r"] - obs
        # TODO: a, safety_dist, obs, b
        hr = h_func(rel_r[0], rel_r[1], a, b, safety_dist)
        return hr
    
    def compute_hd(self, obs):
        rel_r = self.state["r"] - obs
        rd = self.state["rd"] # obs falls out
        term1 = (4 * np.power(rel_r[0],3) * rd[0])/(np.power(a,4))
        term2 = (4 * np.power(rel_r[1],3) * rd[1])/(np.power(b,4))
        return term1+term2

    def compute_A(self, obs):
        rel_r = self.state["r"] - obs
        # print(rel_r)
        A0 = (4 * np.power(rel_r[0], 3))/(np.power(a, 4))
        # print(A0.shape)
        A1 = (4 * np.power(rel_r[1], 3))/(np.power(b, 4))

        return np.array([np.hstack((A0, A1))])

    def compute_h_hd(self, obs):
        h = self.compute_h(obs)
        hd = self.compute_hd(obs)

        return np.vstack((h, hd))
    
    def compute_b(self, obs):
        """extra + K * [h hd]"""
        rel_r = self.state["r"] - obs
        rd = self.state["rd"]
        extra = -(
            (12 * np.square(rel_r[0]) * np.square(rd[0]))/np.power(a,4) + 
            (12 * np.square(rel_r[1]) * np.square(rd[1]))/np.power(b, 4)
        )

        # print("h,hd", self.compute_h_hd(obs))
        b_ineq = extra + self.K @ self.compute_h_hd(obs)
        return b_ineq

    def compute_safe_control(self,obs):
        
        A = self.compute_A(obs)
        # print(A.shape)
        assert(A.shape == (1,2))
        
        b_ineq = self.compute_b(obs)
        #print(b_ineq.shape)
        #print(b_ineq)

        #Make CVXOPT quadratic programming problem
        P = matrix(np.eye(2), tc='d')
        q = -1 * matrix(self.compute_nom_control(), tc='d')
        G = -matrix(A, tc='d')

        h = -matrix(b, tc='d')
        solvers.options['show_progress'] = False
        sol = solvers.qp(P,q,G, h, verbose=False) # get dictionary for solution

        optimized_u = sol['x']
        # optimized_u = self.compute_nom_control()

        return optimized_u
        # u = np.linalg.pinv(A) @ b_ineq

        # return u 

    def compute_nom_control(self, Kn=np.array([-0.08, -0.2])):
        #! mock
        # print('goal:', self.goal)
        # print('state: ', self.state['r'])
        vd = Kn[0]*(self.state['r'] - self.goal)
        # print('vd: ', vd)
        u_nom = Kn[1]*(self.state['rd'] - vd)
        return u_nom

    # def compute_control(self, obs):

@np.vectorize
def h_func(r1, r2, a, b, safety_dist):
    hr = np.power(r1,4)/np.power(a, 4) + \
        np.power(r2, 4)/np.power(b, 4) - safety_dist
    return hr


def main():
    # pass
    dyn = SimpleDynamics()
    ecbf = ECBF_control(dyn.state)

    state_hist = []
    state_hist.append(dyn.state['r'])

    for tt in range(10000):
        # ecbf.plot_h()
        u_hat = ecbf.compute_safe_control(obs=np.array([[0], [0]]))
        #print("U!", u_hat)
        dyn.step(u_hat)
        ecbf.state = dyn.state
        state_hist.append(dyn.state['r'])
        # if(tt % 100 == 0):
        #     print(dyn.state['r'])
        #     print(dyn.state['rd'])
    
    state_hist = np.array(state_hist)
    plt.plot(state_hist[:,0], state_hist[:,1])
    plt.plot(ecbf.goal[0], ecbf.goal[1], '*r')
    plt.plot(state_hist[-1,0], state_hist[-1,1], '*k')
    ecbf.plot_h()
    



if __name__=="__main__":
    main()
