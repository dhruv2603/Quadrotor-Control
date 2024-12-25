# Quadrotor Simulator 

### *RBE502: Robot Control - [Worcester Polytechnic Institute](https://www.wpi.edu/) Fall 2024*

This repository contains a MATLAB simulation for a quadrotor. The simulation allows students to understand and experiment with quadcopter dynamics and control systems.

## Getting Started

### Prerequisites

- MATLAB (R2020a or later recommended)

### Installation

1.unzip the quadrotoro_sim.zip file

2. Open MATLAB and navigate to the `quadrotor_simulator_matlab` directory.

### Running the Simulation

#### Circle Trajectory
1. In the `test_trajectory.m` file, select `trajhandle` as `@circle`.
3. Run the simulator by typing `runsim` in the MATLAB command window and make sure to uncomment the start and stop positions for the circle trajectory and comment all other start and stop positions.


#### Diamond Trajectory
1. In the `test_trajectory.m` file, select `trajhandle` as `@circle`.
2. Run the simulator by typing `runsim` in the MATLAB command window and make sure to uncomment the start and stop positions for the circle trajectory and comment all other start and stop positions.

### Implementing the Controller
1. Select the `controlhandle` as `@pid_controller` or `@lqr_controller` depending on the controller you want to run.
