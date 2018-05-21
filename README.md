
# Overview

This project implements a Model Predictive Controller(MPC) to control a car in Udacity's simulator, to make the car follow the predicted trajectory. The communication strategy is that the simulator send car information to the program via uWebsockets, to get a response from program which contains sterring angle and throttle. 

# Dependencies installation

The project has the following dependencies (from Udacity's seed project):

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's simulator.
- Ipopt and CppAD

And what I want to emphaszie here is that: Please don't try a lot to install the environment on MAC, there are too many strange thing you will encounter：

1. you need to install Openssl again manually.
2. Ipopt can't be installed, finally I followed this dude's [recommendation](https://discussions.udacity.com/t/installing-ipopt-on-mac/502218/12), and got it installed.
3. but when I try out my code, the MPC solve always out put 0 as steering and throttle. I haven't figure out why yet, I suspect that the ipopt solver doesn't work properly, maybe there are still some issues about my installation.


Finally I seted up all the enviroment on a virtual machine with Ubuntu16.04, and it worked. but the simulator runs so badly on vm, so I run my code on unbuntu vm, and simulator on host MAC OS, if anyone will use the same environment, please note it's very important to configure the [Port forwarding](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)


# [Rubic](https://review.udacity.com/#!/rubrics/896/view) points

## Compilation

### Your code should compile.

The code compiles without errors with cmake and make, only a few warning which hs no influence for the demonstration.

## Implementation

### The Model

The model could be described as following state equations:

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] * delta[t-1] * dt / Lf
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] * dt / Lf
```

Where:

- `x, y` : position.
- `psi` : heading.
- `v` : Speed.
- `cte` : Cross-track error.
- `epsi` : Orientation error.
- `Lf`: distance between the car of mass and the front wheels, set as 2.67 here.

The model will try to minimize combination of many factors (see lines 52-72 in MPC.cpp), finally output the car control signal: throttle an steering angle.


### Timestep Length and Elapsed Duration (N & dt)

The number of points(`N`) and the time interval(`dt`) define the prediction horizon and impact the performance of control alot. After tried out of many combination, including N from 10, 20, 30 and dt from 0.1s to 1s. Finally I use 10 as N and 0.1s as dt, it seems work well in simulator for a whole lap. 

### Polynomial Fitting and MPC Preprocessing

The waypoints from simulator need to be transformed to the car coordinate, see lines 111-116 in main.cpp. Then fit to 3-rd degree polynomil, see line 119 in main.cpp. These polynomial coefficients are transmitted to solver to get trajectory.

### Model Predictive Control with Latency

The model used state values which considered the latency, which can be found in lines 122-139 in main.cpp.

## Simulation

### The vehicle must successfully drive a lap around the track.

The model could successfully control the car to drive withoug going off road. Please refer to the [video](https://github.com/FredericLiu/CarND-MPC-P5/blob/master/video/P5-MPC-VIDEO.mov) to see the result.
