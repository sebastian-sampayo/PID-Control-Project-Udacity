# PID Control Project
In this project I implemented a PID controller to maneuver an autonomous driving car around a track in a simulator.
The code is written in C++, because this language provides great performance in terms of memory and speed.
This work is part of the Self-Driving Car Engineer Nanodegree Program at Udacity.

---

[//]: # (Image References)
[simulation]: ./img/simulation.png
[sample1]: ./img/px_vs_py.png
[nis]: ./img/nis.png


## Results

### Video

[A video of the results at 30mph can be found in YouTube in this link.](https://youtu.be/mEsy4CJ_jdI)

[While another video of the results at 50mph can be found in YouTube in this link.](https://youtu.be/eToBo4ScC6Y)

### Simulator
In order to test the controller I used a visualization tool provided by Udacity that simulates the motion of a car around a track. The C++ program receives by telemetry the cross track error (CTE) and the measured speed of the vehicle in real time.

![Simulator][simulation]


### Implementation

For the purpose of the project I implemented a PID controller to calculate the steering angle to keep the CTE as low as possible. The proportional part of the control law make the vehicle turns when it is away from the center of the road. The derivative term provides a better damping so it steers gracefully. Finally, the integral term helps to reduce the steady state error of the system. However, as the CTE measurement is not really noise in this example, this last term isn't very useful.

In order to go further with this project, I also implemented a PID for the speed variable. This way I fixed the desired speed to a certain level and used the measurements to calculate the required throttle so that the speed stayed in the reference level.

The hyperparameters of the system were tunned manually, following the idea of the twiddle algorithm and considering the purpose of each individual term as described above. It's also important to mention that a set of parameters could work really well for low speed, but not so good for high speed. If we expect the vehicle to move fast, then we tune the parameters so that the system can react quickly. This behavior can lead to undesired oscillations at lower speeds. On the contrary, if we tune the parameters so that the system respond smooth and slower it will work well under low speeds, but won't be able to follow the reference trajectory at high speeds. The current set of parameters in the project let the vehicle stay on track for a range of speed between 20 and 50 mph. A little bit of fine tunning could make the vehicle go as fast as 80 mph probably, but it would loose control at 30 mph.



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

