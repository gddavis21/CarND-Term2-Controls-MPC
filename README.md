# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## MPC Control Project Overview

The project assignment was to implement a plug-in algorithm for a vehicle race-track simulator. The primary requirement is to control vehicle speed and direction so that the car drives safely within the confines of the race-track, with a secondary goal of enabling the car to reach relatively high speed (up to 100 MPH) while driving the track.

The plug-in receives periodic messages from the simulator with the following information:
  * Current vehicle position & heading angle relative to map coordinates.
  * Current vehicle speed.
  * A sequence of "waypoints" (also in map coordinates) representing the race-track trajectory immediately ahead of the vehicle (approximately 100 meters).
  * Current actuator values for steering angle and throttle.

The plug-in is required to utilize a Model Predictive Control (MPC) algorithm to continuously update the steering and throttle actuators.

An additional challenge is the introduction of (simulated) actuator latency. The plug-in is required to delay 100ms before sending the new actuator values to the simulator. The control algorithm must compensate for latency so that the vehicle stays safely on the road.

## Running the MPC Controller plug-in app

The simulator runs concurrently with the project plug-in app "mpc".

By default running ./mpc will drive the vehicle at a target velocity of 90 MPH, with visualization turned OFF.

There are 2 optional command-line arguments:
  1. Run ./mpc nnn to run a different target velocity, where nnn is the target velocity in miles per hour. For example, run ./mpc 55 to run at 55 MPH target velocity. The velocity is clipped to the range [10,100] MPH.
  2. Enable visualization by running ./mpc 55 vis (in this case 55 is just an example, you can set any target velocity). In this mode the simulator displays a visualization of the reference and predicted trajectories for the vehicle.

## Explanation of MPC Control

Model Predictive Control solves the problem of updating 1 or more independent variables in order to maintain (near) optimal system state, given current state and a model of how the state will change over (near-term) time with changes to the independent variables.

MPC is framed as an optimization problem that solves for the values of each independent variable over some number of near-future time steps. The cost to be minimized includes a measure of how much the modeled system would deviate from optimal state, as well as additional cost terms designed to keep the control system within desired operating conditions.

The critical aspect of MPC is that it only actuates the optimal solution for the next time step, after which current state is re-measured and the MPC optimization starts again. In this way the algorithm is able to account for the near-term future ("receding horizon") in its predictions, but remains as responsive as possible to the changing state of the system.

## Project MPC Model Description

For this project I implemented MPC on a straightforward kinematic model for vehicle motion, as taught in the course lectures.

The model actuators (independent variables) are steering angle and throttle value. Positive throttle value equates to acceleration, and negative throttle value equates to deceleration (braking) of the vehicle.

The model state includes vehicle pose (x/y position and orientation angle), current velocity, and estimates of cross-track error and orientation error.

Model state update equations are as follows:
  * Given initial vehicle state at time t = t_0:
    - initial position (x_0, y_0)
    - initial orientation psi_0
    - initial velocity v_0
  * Also given initial actuator values:
    - initial steering angle delta
    - initial acceleration a
  * Also given a calibration parameter Lf that quantifies the relationship between vehicle length and turn radius
  * Compute updated state at time t = t_0 + dt:
    - x = x_0 + (v_0 * cos(psi_0) * dt)
    - y = y_0 + (v_0 * sin(psi_0) * dt)
    - psi = psi_0 + ((v_0/Lf) * delta * dt)
    - v = v_0 + (a * dt)

Computing reference-state error:
  * Vehicle cross-track error (CTE) is the (signed) distance from vehicle position to the reference trajectory curve. CTE is estimated by the difference between initial y-position and the reference trajectory polynomial evaluated at the initial x-position.
  * Vehicle orientation error is the difference between vehicle orientation and the tangent angle to the reference trajectory curve. We estimate the reference tangent angle by computing the slope of the tangent (1st derivative of the reference trajectory polynomial evaluated at initial x-position), and applying the arc-tangent function.

## MPC Algorithm Overview

  * Given map-relative track waypoints, vehicle pose & velocity, and current steering angle and throttle values.
  * Data pre-processing:
    + Transform waypoints into vehicle coordinates. The coordinate frame is zeroed at the vehicle location, with the x-axis aligned to vehicle orientation.
    + Best-fit a cubic polynomial to vehicle-relative waypoints, using standard polynomial regression. The resulting polynomial is used as the reference (desired) trajectory for the vehicle.
    + Transform vehicle state to vehicle coordinates. This is easy--vehicle position and orientation are simply set to zero!
    + Compensate vehicle state for actuator latency.
  * Apply MPC optimization to compute steering angle & throttle:
    + Compute initial cross-track error and orientation error.
    + Setup and solve MPC optimization problem.
  * Return optimized 1st time-step actuator values to simulator.
    
## Implementation Details

### Choosing time-step duration, number of steps, etc:

An important part of setting up and solving the MPC optimization problem is choosing the time-step duration (dt) and the number of time steps to solve for (N). The number of time steps determines the number of variables solved for by the optimizer, and the time-step duration is a free parameter used in all the model update equations. 
  
More time steps mean more computation time to solve the MPC optimization problem. Since the algorithm must run quickly, the number of time steps must be limited to a number appropriate to available compute resources.
  * I found that running the optimization solver over N=20 time steps results in a reasonable compute load on my development PC.
  * I did test at other values like 10 or 15 time steps. At higher speeds the controller worked better with more time steps, so I left it at 20.
  * In a real-world setting I think it would be important to calibrate compute performance and adjust this parameter accordingly.
  
For time-step duration, I found it was more effective to directly set a related parameter and let time-step duration be determined by that. The related parameter is what I called "distance to horizon" (H), which is simply the distance over which the MPC optimization problem is solved. 
  * Because the simulator constantly supplies the upcoming road trajectory (of about 100 meters), I decided to setup the MPC optimization to always solve over a distance just short of that (H = 80 meters).
  * Distance to horizon satisfies the equation H = N\*v\*dt, where v is the current vehicle velocity. Thus dt = H/(N*v) is recomputed as a function of current velocity each time we solve the MPC optimization.

### MPC Optimization cost function & parameter tuning:

The MPC optimization cost function is a simple sum of weighted squares.
  * Minimize error:
    - Sum of weighted squared cross-track errors
    - Sum of weighted squared orientation errors
    - Sum of weighted squared deviations of velocity from reference velocity
  * Smooth/penalize actuator changes:
    - Sum of weighted squared steering angle values
    - Sum of weighted squared throttle values
  * Smooth/penalize rate of actuator changes:
    - Sum of weighted squared steering changes (between consecutive time steps)
    - Sum of weighted squared throttle changes (between consecutive time steps)

In summary, the cost function is designed to compute small magnitude, smoothly changing actuation values that maintain small errors in position, orientation and velocity. Other than the weight values, this cost function is identical to what was recommended in the MPC lesson and practice assignment. 

Choosing and tuning the cost function weight parameters is a critical aspect of achieving good algorithm performance. I manually tuned all parameters.
  * I found effective fixed weights for velocity error, throttle value, and rate of throttle change. These values worked well over the full range of target velocities I tested at (30-100 MPH).
  * I found it was helpful to progressively weight the cross-track and orientation error terms, giving more weight to the "near-future" terms and less weight to the "far-future" terms.
  * I found the determination of weights for the steering value and rate of steering change terms is highly dependent on current vehicle velocity.
    - The weights scale up with velocity in a non-linear relationship.
    - I found it effective to use the same value for both weights. I don't have a good explanation for why, and it could turn out with further testing & tuning that different values are better.
    - The algorithm recomputes these weights on each iteration of the MPC optimization, given the current vehicle velocity.
      + I manually tuned the weight parameter at several different speeds.
      + The algorithm re-computes the weights using a linear interpolation over the manually tuned (velocity, weight) samples.
      + I also tried using quadratic and cubic polynomials fit to the (velocity, weight) samples, and using the fitted polynomial to compute weights. However, I didn't have a good justification for assuming it's a quadratic or cubic polynomial relationship, so in the end I decided to go with the non-parametric approach.

### Accounting for actuator latency:

The project requires simulating actuator latency by delaying for 100ms before sending the computed actuator values back to the simulator.

My implementation compensates for latency by modeling the change in vehicle state over the latency time before solving the MPC optimization.
  - Start with current vehicle state and actuator values reported by the simulator (call this time t0).
  - Compute vehicle state at time t = t0 + latency (using the kinematic model update equations).
  - Use this latency compensated vehicle state as the initial input state to the MPC optimization.

My reasoning for this technique is as follows:
  - Due to latency, the simulator will be delayed in applying the actuator changes computed by MPC.
  - The algorithm can model this latency by solving the MPC problem over a future time window. The time window we want to solve for is offset into the future by the latency time. 
  - Thus we need to "seed" the solver with the (approximate) vehicle state at time t = now + latency.

## Other observations/reflections:

One significant difficulty with the project is that on my development PC the simulator sends messages to the plug-in at significantly different update rates, depending on the window size chosen for the simulator (window size is chosen when you start the simulator app). At 640x480 the update rate is 3-4 messages per second, but at higher window sizes the update rate falls below 1 message per second. At the higher window sizes, my algorithm loses control of the vehicle at higher speeds. It's clear that the effectiveness of the algorithm is dependent on update frequency, especially as the vehicle speed increases.

My main reservation with the techniques used in this project are around the use of simple cubic polynomials to model the reference trajectory.
  * The explicit y=f(x) polynomial approximation is adequate as long as the trajectory remains relatively parallel to the vehicle orientation.
  * However, problems occur if/when the trajectory diverges from vehicle orientation. This happens when either the road has a sharp turn or bend, or the vehicle (wrongly) veers away from reference trajectory. In these cases the "slope" of the cubic polynomial becomes steep, leading to numerical instability (or undefined behavior!) in the explicit y=f(x) function.
  * A much better solution for the trajectory would be to compute a parametric curve (B-spline, Bezier spline, etc.) or non-parametric curve (smoothing spline, thin-plate spline). These are designed to model 2D curves without the difficulties of using explicit functions. Of course, they're also more complex mathematically and were definitely out-of-scope for completing this project on time.

Overall, I was very impressed with the effectiveness of the MPC approach compared with the PID controller approach we implemented for the previous project. I was able to safely "drive" my vehicle at 3-4x the speed with much less time spent tuning the controller.

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

Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
