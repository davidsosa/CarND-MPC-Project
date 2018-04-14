## CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
## Project completed by David Sosa. 11.04
---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


## Background of the project

For this project we apply the a kinematic model (as opposed to a more complicated dynamic model), where
the actuators are only the acceleration and the steering.

The kinematic model equations are the following:

x(t+1) = x(t) + v(t)cos(psi(t)) * dt
y(t+1) = y(t) + v(t)sin(psi(t)) * dt
psi(t+1) = psi + vt/Lf * delta(t) * dt
v(t+1) = v(t) = v(t) + a(t) * dt,

where delta and a are the actuators steering and acceleration and Lf is the measurement
of the distnce between the center of mass of the vehicle and it's front axle. The larger
the vehicle, the slower the turn rate. The value is already specified for the project. 

Errors to minimize

Cross track error:

cte(t) = y(t) - f(x)

cte(t+1) = cte(t) + v(t)*sin(epsi)*dt

therefore:

cte(t) = y(t) - f(x) + v(t)*sin(epsi)*dt


The orientation error is given by, 
epsi(t+1) = epsi(t) + vt/Lf * delta(t) * dt
The current epsi(t) is the difference of the current psi and the desired psi.




## First task

Extract the values from the json files. 



## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
