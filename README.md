# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

1. The ModelStudent describes their model in detail. This includes the state, actuators and update equations.The ideal path (also called waypoints/reference line or “shape of the road ahead”)  is provided by the simulator as a set of points but could be extracted from a video image using computer vision or extracted from map data using localization algorithm. The algorithm uses these waypoints to generate a polynomial using polyfit. This generates a set of coefficient. Here a third order polynomial has been used as it is sufficient for the complexity of our road. The MPC takes the state of the vehicle and the coefficients of the polynomial describing the shape of the road ahead and optimizes the predicted path using actuator values that have been constrained to realistic value (for example, the .steerinng actuator cannot steer more than 25 degrees.) The model implemented was the following:  ```cpp
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);fg[1 + psi_start + t] = psi1 - (psi0 - v0/Lf * delta0 * dt);fg[1 + v_start + t] = v1 - (v0 + a0 * dt);fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0/Lf * delta0 * dt);
```### Constraints : Constraints on other variables (like state) have been set to arbitrarily high range (unconstrained) *Steering : constrained between 0-25 degrees on each side*Accelerator: constrained between -1 and 1 (units relative to simulation software). To optimize the problem the model tries to minimize a cost term, which is a function of the cross-track error, angle error, acceleration and jerkyness (variation of the acceleration).  We also added penalty for a combination of velocity and steer and we could have added a change of sign in steering, which will lead to a weight transfer on a real car. 2. Timestep Length and Elapsed Duration (N & dt)The number of timesteps choosen (N) was set to 10 and the number of time elapsed between steps (dt) was set to 0.1. These value come from the results of other students put either on the udacity forums or on github. I originally tried N = 25 and dt = 0.05 as this was the default in the template for the project, but it performed better with the formerly stated values, especially to simultaneously record screen video,.3. Polynomial Fitting and MPC PreprocessingA polynomial is fitted to waypoints.If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.4. Model Predictive Control with LatencyThe student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

# Original Udacity Readme 

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
