import numpy as np
import matplotlib


def main():
    print("start")


    # Initialize
    x = np.array([0,0,10])
    xdot = np.zeros_like(x)
    theta = np.zeros_like(x)
    thetadot = np.zeros_like(x)

    # Step through simulation
    for t in range(100):
        # Get control input
        u = calc_control()

        # Compute angular velocity vector from angular velocities
        omega = thetadot2omega(thetadot, theta)

        # Compute angular and linear accelerations given input and state
        # TODO: combine state
        a = acc(u, theta, xdot, m, g, k, kd)
        omegadot = ang_accel(u, omega, I, L, b, k)

        # Compute velocity and state
        omega = omega + dt * omegadot
        thetadot = omega2thetadot(omega, theta)
        theta = theta + dt * thetadot
        xdot = xdot + dt * a
        x = x + dt * xdot
        print(x)

def compute_thrust(u,k):
        """compute thrust from input and thrust coefficient"""
        T = np.array([0, 0, k*np.sum(u)])

def calc_torque(u, L, b, k):
        
if __name__ == '__main__':
    main()
