
# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

## Background
Unscented Kalman Filter can be used to track moving objects from a Self Driving car using multiple sensors, in our case, LIDAR and RADAR measurements. It does not make linear approximation for the angular velocity like the Extended Kalman Filter, and is thus more accurate in predicting sharp vehicle turns.
Several object motion models  can be used, in this case, we will use the constant turn rate and velocity magnitude model (CTRV). 
![CTRV](https://raw.githubusercontent.com/eshnil2000/CarND-Unscented-Kalman-Filter-Project/master/images/ctrv.png)

## Unscented Kalman Filter Intuition (From https://www.haidynmcleod.com/unscented-kalman-filter)

The Object motion is represented and tracked by a state vector, made up of Initialize state x_ vector:
px - position in x, units meters
py - position in y, units meters
v - longitudinal velocity [ x & y components calculated geometrically], units meters/s
psi - yaw angle to X axis units radians
psi_dot - rate of change of yaw angle , units radians/s

```
*/
//set example state to all 1s, seems within range of reasonable values for position, velocity, yaw rate
      
      x_ <<  1,
             1,
             1,
             1,
             1; 

// init covariance matrix. Set diagonals to non negative values representing variance 
//square of standard deviation
//Since we have standard deviation of laser sensor as 0.15, set this variance to 0.15*0.15=0.0225, good first estimate of the position variances.
             
      P_ <<    .0225, 0, 0, 0, 0,
               0, .0225, 0, 0, 0,
               0, 0, 1, 0, 0,
               0, 0, 0, 1, 0,
               0, 0, 0, 0, 1; 

```


Prediction - A prediction of the state vector at the next timestep is made using non-linear function. Since function is non-linear, the state distribution will not be Gaussian. Instead of analytically solving the distribution, the UKF method instead generates "Sigma" Points to represent mean and standard deviation , and then transforms each of these points through the non-linear function to estimate the distribution.

process noise which models the longitudinal acceleration and the radial acceleration are added to the x_ vector, to get an "Augmented" vector. 

Once again, the Sigma points are generated, now with 7 rows instead of the 5 in x_ originally. 

Now predict the next Sigma points using the non0linear Object motion process model. 
```
// predict the next timestep sigma points using the process model 
        pred_sigma_pts(0,i) = pos_x + 0.5 * a_pos * delta_t * delta_t * cos(yaw);
        pred_sigma_pts(1,i) = pos_y + 0.5 * a_pos * delta_t * delta_t * sin(yaw);
        pred_sigma_pts(2,i) = vel + a_pos * delta_t;
        pred_sigma_pts(3,i) = yaw + yaw_dot * delta_t + 0.5 * a_yaw_dot * delta_t * delta_t;
        pred_sigma_pts(4,i) = yaw_dot + a_yaw_dot * delta_t;
```

Calculate the mean and covariance of these points.

At this time, all predictions have been done using the Non-linear process model.
Now, take a measurement. Apply a prediction from the measurement. Note: measurement doesn't require augmentation, since sensor noise is additive.

Now we have measured mean and variance, as well as the predicted mean and variance.
The updated state vector  will be : 
predicted state vector + Kalman gain * measured state vector 
[Measured State vector will include a measurement at the current timestamp as well prediction of measurement from the previous timestamp]

The Kalman gain is the Cross Correlation between the sigma points in state space and measurement space.



Update


## Unscented Kalman Filter Algorithm Psuedo-Code:
Prediction:


Measurement Update:


Intuition: 


## Input Data File Description
See https://github.com/eshnil2000/CarND-Unscented-Kalman-Filter-Project/blob/master/README.md Input Data File Description


## Goals
See the rubric file below for goals, to maintain the RMSE below the tolerated.

## Unscented Kalman Filter Implementation Algorithm:
Here is the output video of the Kalman filter
![Kalman Video](https://raw.githubusercontent.com/eshnil2000/Udacity-CarND-Extended-Kalman-Filter-Project/master/images/kalman.gif)


###################################################################################

# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, src/ukf.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

