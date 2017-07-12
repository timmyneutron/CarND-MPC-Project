![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)

# Udacity Self-Driving Car Nanodegree: Term 2
# Project #5: Model Predictive Control

![](https://raw.githubusercontent.com/timmyneutron/CarND-MPC-Project/master/img/mpc_project_screenshot.png)

## Introduction
This is a project for Udacity's Self-Driving Car Nanodegree. It implements a model predictive controller (MPC) to steer a car around a simulated track at high speed.

## Concepts
Concepts explored in this project:

  - Vehicle kinematic and dynamic models
  - Actuator constraints
  - Curve fitting and trajectory planning
  - Latency
  - Non-linear optimization using Ipopt and CppAD libraries

## Getting Started
To watch a video of the controller in action, click [here](https://youtu.be/IpkLxuv-udc).

To build and run the project:

  - Click [here](https://github.com/udacity/self-driving-car-sim) for instructions on installing the simulator
  - Click [here](https://github.com/udacity/CarND-MPC-Project) for instructions on installing dependencies and running the controller

Source code is located in the `src` folder.

## Write-Up

### Model
The state vector of the model is comprised of the x and y coordinates of the car, along with the yaw angle, ![](https://raw.githubusercontent.com/timmyneutron/CarND-MPC-Project/master/img/psi.png), and the velocity of the car. It also assumes two control inputs: acceleration and the steering angle, ![](https://raw.githubusercontent.com/timmyneutron/CarND-MPC-Project/master/img/delta.png). ![](https://raw.githubusercontent.com/timmyneutron/CarND-MPC-Project/master/img/Lf.png) represents the distance from the center of gravity to the front of the car.

The model update equations are:

![](https://raw.githubusercontent.com/timmyneutron/CarND-MPC-Project/master/img/eqn1.png)

![](https://raw.githubusercontent.com/timmyneutron/CarND-MPC-Project/master/img/eqn2.png)

![](https://github.com/timmyneutron/CarND-MPC-Project/blob/master/img/eqn3.png?raw=true)

![](https://github.com/timmyneutron/CarND-MPC-Project/blob/master/img/eqn4.png?raw=true)

### Timestep Length and Elapsed Duration

After trying many values (ranging from 10 to 50 timesteps, and a dt between 0.01 and 0.1 seconds), I settled on using 20 timesteps with a dt of 0.04 seconds. There are many tradeoffs to this choice - a smaller dt means the actual performance of the car will deviate less from the predicted trajectory, but it reduces the amount of time the car predicts into the future. Likewise, increasing the number of time steps used helps to predict the car's behavior further into the future, but also contributes to latency, which negatively affects the predictive power of the model.

20 timesteps with a dt of 0.04 seconds seems to be the right balance between these tradeoffs, and allows the car to maintain control at speeds of up to 70 mph.

### Polynomial Fitting and MPC Preprocessing
The simulator reports the x and y coordinates and yaw angle of the  car and the waypoints of the track in the global frame, and these must be converted to the car's reference frame, where the x axis is straight ahead of the car, and the y axis is to the left. Then the midline of the track is estimated using a 3rd-order polynomial regression on the waypoints. Using this, the MPC can calculate the ideal trajectory for the car.

### Model Predictive Control with Latency
The project simulates a 0.1 s latency between a control signal being sent and actuation. To account for this, the MPC controller estimates the car's position 0.1 s into the future by assuming constant velocity and no turn rate (a first order approximation, but adequate for the purposes of this project).

The controller then considers a large number of possible trajectories based on set constraints, such as constraining the car to stay on the track, and constraining the predicted trajectories to obey the update model listed above.

A cost function is set to include three variables to minimize: the difference between the desired speed and actual speed, the steering rate, and the acceleration rate (or jerk). Minimizing these assures the smoothest ride possible.

Once the cost function is set, the MPC finds the trajectory that minimizes this cost function over that whole trajectory, and returns vectors for each of the state variables and actuators.

The x and y values for the predicted trajectory are used to draw the green line in the sim, and the values the MPC returns the steering angle and acceleration values that are used to control the car.
