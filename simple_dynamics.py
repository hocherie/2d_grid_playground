import numpy as np
import matplotlib.pyplot as plt

class SimpleDynamics():
    def __init__(self):
        ## State space
        r = np.zeros((2,1)) # position
        rd = np.zeros((2,1)) # velocity 
        self.state = {"r":r, "rd":rd}
        ## Params
        self.dt = 10e-3

        # self.u = zeros(2,1) # acceleration, control input
    
    def step(self, u):
        # rdd = self.u
        rd = state["rd"] + self.dt * u 
        r = state["r"] + self.dt * state["rd"]

        state["rd"] = rd
        state["r"]  = r

class ECBF_control():
    def __init__(self, state):
        self.state = state
        self.shape_dict = {} #TODO: a, b
        self.gain_dict = {} #TODO: Kp, Kd
        # pass

    def plot_h(self):
        obs = np.array([0,0]) #! mock
        
        plot_x = np.arange(-5, 5, 0.1)
        plot_y = np.arange(-5, 5, 0.1)
        xx, yy = np.meshgrid(plot_x, plot_y, sparse=True)
        z = h_func(xx,yy, 1, 1, 1) > 0
        # z[z<0] = 0# make unsafe -199
        # print(z)
        # z = np.sin(xx**2 + yy**2) / (xx**2 + yy**2)
        h = plt.contourf(plot_x, plot_y, z, [-1, 0, 1])
        proxy = [plt.Rectangle((0, 0), 1, 1, fc=pc.get_facecolor()[0])
                 for pc in h.collections]
        plt.legend(proxy, ["Unsafe: range(-1 to 0)","Safe: range(0 to 1)"])

        
        plt.show()



    def compute_h(self, obs=np.array([0, 0])):
        rel_r = self.state["r"] - obs
        # TODO: a, safety_dist, obs, b
        hr = self.h_func(rel_r[0], rel_r[1], a=1, b=1, safety_dist=1)
        return hr
    
    def compute_hd(self, obs):
        rel_r = self.state["r"] - obs
        rd = self.state["rd"] # obs falls out
        term1 = (4 * rel_r[0]^3 * rd[0])/(np.power(a,4))
        term2 = (4 * rel_r[1] ^ 3 * rd[1])/(np.power(b,4))
        return term1+term2

    def compute_A(self, obs):
        rel_r = self.state["r"] - obs
        A0 = (4 * rel_r[0] ^3)/(np.power(a,4))
        A1 = (4 * rel_r[1] ^ 3)/(np.power(b,4))

        return np.hstack((A0, A1))

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

        b_ineq = extra + K @ self.compute_h_hd(self, obs)
        return b_ineq

    def compute_safe_control(self,obs):
        A = self.compute_A(obs)
        b_ineq = self.compute_b(obs)

        u = np.linalg.pinv(A) @ b_ineq

        return u 

    def compute_nom_control(self):
        #! mock
        return np.array([1,0])

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
    ecbf.plot_h()


if __name__=="__main__":
    main()
