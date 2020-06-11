# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Introduction

In this project, I implement a PID controller in C++ to maneuver the vehicle around the lake race track from the Behavioral Cloning Project. A cross track error is a distance of the vehicle from trajectory.

## Implementation

PID is made up of three components:

1. P component - the steering angle in proportion to CTE with a proportional factor tau.

```
-tau * cte
```

2. I component - integral or sum of error to deal with systematic biases.

```
int_cte += cte
tau_i * int_cte
```

3. D component - the differential component of the controller which helps to take temporal derivative of error.

```
diff_cte = cte - prev_cte
prev_cte = cte
-tau_d * diff_cte
```

## Reflection

1. Describe the effect each of the P, I, D components had in your implementation.



2. Describe how the final hyperparameters were chosen.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
