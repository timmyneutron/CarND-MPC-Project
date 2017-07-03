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

## Viewing the Project
To watch a video of the controller in action, click [here](https://www.dropbox.com/s/le11wa9p0wbv7qt/CarND-MPC-Project-Video.mov?dl=0). The yellow path represents the midline of the road, and the green path represents the optimal trajectory calculated by the controller.

Source code is located in the `src` folder.

## Write-Up

### Model
The state vector of the model is comprised of the x and y coordinates of the car, along with the yaw angle, psi, and the velocity of the car. It also assumes two control inputs: acceleration and steering angle, delta.

The model update equations are:

![](https://raw.githubusercontent.com/timmyneutron/CarND-MPC-Project/master/img/eqn1.png)

![](https://github.com/timmyneutron/CarND-MPC-Project/blob/master/img/eqn2.png?raw=true)

![](https://github.com/timmyneutron/CarND-MPC-Project/blob/master/img/eqn3.png?raw=true)

![](https://github.com/timmyneutron/CarND-MPC-Project/blob/master/img/eqn4.png?raw=true)

### Timestep Length and Elapsed Duration

After trying many values (ranging from 10 to 50 timesteps, and a dt between 0.01 and 0.1 seconds), I settled on using 20 timesteps with a dt of 0.04 seconds. There are many tradeoffs to this choice - a smaller dt means the actual performance of the car will deviate less from the predicted trajectory, but it reduces the amount of time the car predicts into the future. Likewise, increasing the number of time steps used helps to predict the car's behavior further into the future, but also contributes to latency, which negatively affects the predictive power of the model.

20 timesteps with a dt of 0.04 seconds seems to be the right balance between these tradeoffs, and allows the car to maintain control at speeds of up to 70 mph.

### Polynomial Fitting and MPC Preprocessing
The simulator reports the x and y coordinates and yaw angle of the  car and the waypoints of the track in the global frame, and these must be converted to the car's reference frame, where the x axis is straight ahead of the car, and the y axis is to the left. Then the midline of the track is estimated using a 3rd-order polynomial regression on the waypoints. Using this, the MPC can calculate the ideal trajectory for the car.

### Model Predictive Control with Latency
The MPC considers a large number of possible trajectories based on set constraints, such as constraining the car to stay on the track, and constraining the predicted trajectories to obey the update model listed above.

A cost function is set to include three variables to minimize: the difference between the desired speed and actual speed, the steering rate, and the acceleration rate (or jerk). Minimizing these assures the smoothest ride possible.

Once the cost function is set, the MPC finds the trajectory that minimizes this cost function over that whole trajectory, and returns vectors for each of the state variables and actuators.

The x and y values for the predicted trajectory are used to draw the green line in the sim, and the values the MPC returns for delta and a are used to control the car.

Because of the 0.1 s latency, the MPC returns the delta and a values for 2 timesteps in the future to approximate the appropriate control at that time.