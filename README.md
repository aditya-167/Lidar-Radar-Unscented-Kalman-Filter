# Unscented kalaman filter with Constant Turn Rate Velocity Process model with Lidar and Radar sensor data fusion

<img src="media/ukf.gif" width="900" height="500" />

In this project we will implement an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy lidar and radar measurements and  obtaining RMSE values that are lower that the tolerance outlined.

The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ukf_highway

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, and src/ukf.h

## UKF Pipeline:

Process Models:  

- [CTRV (constant turn rate and velocity magnitude)](#ctrv-constant-turn-rate-and-velocity-magnitude)

Measurement Models:  

- [Radar](#radar) 
- [Lidar](#lidar)


### Code files

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. 
The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the 
other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has
it's own UKF object generated for it, and will update each indidual one during every time step. 

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so we are only tracking along the X/Y axis.

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
 * PCL 1.2

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ukf_highway`



## Unscented Kalman Filter
The equations below describe the unscented Kalman filter and are implemented in `src/ukf.cpp` 
![Algorithm_Unscented_Kalman_filter](docs/pics/Algorithm_Unscented_Kalman_filter.png)
For explanations of what each variable means, please, refer to comments in the code in corresponding files or 
the book "Thrun, S., Burgard, W. and Fox, D., 2005. Probabilistic robotics. MIT press."

## Process Models

### CTRV (Constant Turn Rate and Velocity Magnitude)
The CTRV process model is a process model where we assume the object moves with a constant turn rate and velocity,i.e with zero longitudinal and yaw accelerations. 
The state vector consists of 5 components ( px, py, v, yaw, yaw_rate ) where p is the position, 
v represents the velocity module, yaw represents 
the [yaw angle](https://en.wikipedia.org/wiki/Aircraft_principal_axes), and yaw_rate represents the yaw velocity. The leftmost column in the following equation represents the non-linear process noise; 
a_a represents longitudinal acceleration, and a_psi is yaw acceleration.

![CTRV_process_model](docs/pics/CTRV_process_model.png)  
where 
![CTRV_process_model_alpha](docs/pics/CTRV_process_model_alpha.png)
and
![CTRV_process_model_beta](docs/pics/CTRV_process_model_beta.png)


## Measurement Models

### Lidar

The measurement vector consists of 2 components ( px, py ) where p\* represents the position. 
The transformation from the state space to the Lidar measurement space is as follows  
![Lidar_measurement_model](docs/pics/Lidar_measurement_model.png)
where
![Lidar_measurement_model_H](docs/pics/Lidar_measurement_model_H.png)
The Lidar measurement model is implemented as 
a [LidarMeasurementModel](src/measurement_models/LidarMeasurementModel.hpp) class.

### Radar
The Radar measurement model is a non-linear measurement model. The measurement vector consists of 3 components ( range,bearing, range_rate ) where the range is a radial distance from the origin, 
 the bearing is an angle between range and X-axis which points into the direction of the heading of the vehicle, range_rate is a radial velocity. 
 The transformation from the state space to the Radar measurement space is as follows  
![Radar_measurement_model](docs/pics/Radar_measurement_model.png)
where
![Radar_measurement_model_h](docs/pics/Radar_measurement_model_h.png)

All the sensor update and prediction step can be found in `src/ukf.cpp`.


