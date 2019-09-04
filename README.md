# 2d_grid_playground
2D Grid Environment with common utils (raytracing) and quadrotor dynamics for quick prototyping. Includes following files:

* `main.py`: Simulates quadrotor maneuvering in 2D grid with 2nd order dynamics executing naive safe control.

* `sim_utils.py`: Contains common utility functions for simulator. Ex. `get_rot_matrix(angles)`

* `simulator.py`: Creates 2D grid simulator and enables basic range sening. Contains Map class (create from txt file), Robot class (stores current and paast state, also instantiates QuadDynamics object)

* `controller.py`: Controller-related functions for quadrotor cascaded control. (ex. Position, Velocity, Attitude Controller). Mainly use by calling `go_to_position(state, des_pos, param_dict)`.

* `dynamics.py`: Contains QuadDynamics class which gives a simple 3d quadrotor dynamics given 2nd order equations of motion. Use by instantiating class and calling `self.step_dynamics(state, u)` to update quadrotor state. Based on http://andrew.gibiansky.com/downloads/pdf/Quadcopter%20Dynamics,%20Simulation,%20and%20Control.pdf


* `visualize_dynamics.py`: Contains graphing-related functions for dynamics.py. Mainly use for tuning PID controllers.







# Author
Cherie Ho
