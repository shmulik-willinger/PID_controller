# PID controller

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---

In this project the goal is to implement a PID controller in C++. The communication between the project and the Udacity simulator is done using WebSocket, where the simulator provides cross-track error (CTE), speed, and steering angle data. The controller must respond with steering and throttle data to reliably drive a car around the track simulator.

![]( https://github.com/shmulik-willinger/PID_controller/blob/master/readme_img/sample1.jpg?raw=true)

## PID controller

A `Proportional–Integral–Derivative controller` (PID controller) is a control loop feedback mechanism widely used in industrial control systems and a variety of other applications requiring continuously modulated control.

A PID controller continuously calculates an error value as the difference between a desired setpoint and a measured process variable, and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively) which give the controller its name.

The controller consist of the following components:

The `proportional` component tries to steer the car toward the center line, and has the most directly observable effect on the car's behavior. If the car is far to the right it will steer hard to the left, and vice versa.

The `integral` component comes to compensate systematic bias. It tries to eliminate a possible bias on the controlled system that could prevent the error to be eliminated. If used along, it makes the car go in circles.

The `differential` section helps to avoid the over-shooting, by smoothing the approach to it. As the car drives towards the CTE, the differential term goes smaller.

![]( https://github.com/shmulik-willinger/PID_controller/blob/master/readme_img/sample2.jpg?raw=true)
(The image was taken from a video by AerospaceControlsLab)

Prerequisites and Executing
---

This project requires the following dependencies:

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

Build Instructions:

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`

Running the project:
1. Run the project main file: `./PID `
2. You will get the following output:
 `Listening to port 4567. Connected!!! `
 3. Run the Simulator on `project 4` and start the process.

Simulation
---

The following videos demonstrates the difference in performance when running the simulator against a controller that dosen't support ALL the PID components. The final PID controller implementation performed well, and the vehicle successfully drive a lap around the track.

![]( https://github.com/shmulik-willinger/PID_controller/blob/master/readme_img/PID_converges.jpg?raw=true)

P controller               |  PD controller
:---------------------:|:---------------------:
![]( https://github.com/shmulik-willinger/extended_kalman_filter/blob/master/readme_img/radar_only.jpg?raw=true)  |  ![]( https://github.com/shmulik-willinger/extended_kalman_filter/blob/master/readme_img/laser_only.jpg?raw=true)

PI controller               |  PID controller
:---------------------:|:---------------------:
![]( https://github.com/shmulik-willinger/extended_kalman_filter/blob/master/readme_img/radar_only_2.jpg?raw=true)  |  ![]( https://github.com/shmulik-willinger/extended_kalman_filter/blob/master/readme_img/laser_only_2.jpg?raw=true)

Its easy to see that the PID controller converges better than the others controllers. 


Process results
---

The vehicle successfully drive a lap around the track.
