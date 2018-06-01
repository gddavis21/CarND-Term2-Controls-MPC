# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## MPC Control Project Overview

This project entails implementing a plug-in algorithm for a vehicle race-track simulator. The primary objective is to control vehicle speed and direction so that the car drives safely within the boundaries of the race-track (as opposed to driving off the road). The secondary objective is to enable driving at relatively high speed (up to 100 MPH).

The plug-in receives periodic messages from the simulator with the following information:
  * Current vehicle position & heading angle relative to map coordinates.
  * Current vehicle speed.
  * Sequence of "waypoints" (also in map coordinates) representing the race-track trajectory immediately ahead of the vehicle (approximately 100 meters).
  * Current actuator values for steering angle and throttle.

The plug-in is required to utilize a Model Predictive Control (MPC) algorithm to continuously update the steering and throttle actuators.

An additional challenge is the introduction of (simulated) actuator latency. The plug-in is required to delay 100ms before sending the new actuator values to the simulator. The control algorithm must compensate for latency so that the vehicle stays safely on the road.

## Explanation of MPC Control

Model Predictive Control solves the problem of updating 1 or more independent variables in order to maintain (near) optimal system state, given current state and a model of how the state will change over (near-term) time with changes to the independent variable.

MPC is framed as an optimization problem that solves for the values of each independent variable over some number of near-future time steps. The cost to be minimized includes a measure of how much the modeled system would deviate from optimal state, as well as additional cost terms designed to keep the control system within desired operating conditions.

The critical aspect of MPC is that it only actuates the optimal solution for the next time step, after which current state is re-measured and the MPC optimization starts again. In this way the algorithm is able to account for the near-term future ("receding horizon") in its predictions, but remains as responsive as possible to the changing state of the system.

## Project MPC Model Description

For this project I implemented MPC on a straightforward kinematic model for vehicle state, as demonstrated in the course lectures.

The model actuators (independent variables) are steering angle and throttle value.

The model state includes vehicle pose (x/y position and orientation angle), current velocity, and estimates of cross-track error and orientation error.
  * Vehicle cross-track error (CTE) is the (signed) distance from vehicle position to the reference trajectory curve. CTE is estimated by the difference between initial y-position and the reference trajectory polynomial evaluated at the initial x-position.
  * Vehicle orientation error is the difference between vehicle orientation and the tangent angle to the reference trajectory curve. We estimate the reference tangent angle by computing the slope of the tangent (1st derivative of the reference trajectory polynomial evaluated at initial x-position), and applying the arc-tangent function.

Model state update equations are as follows:
  * x = x_0 + (v_0 * cos(psi_0) * dt)
  * y = y_0 + (v_0 * sin(psi_0) * dt)
  * psi = psi_0 + ((v_0/Lf) * s * dt)
  * v = v_0 + (a * dt)

## MPC Algorithm Overview

  * Given map-relative track waypoints, vehicle pose & velocity, and current steering angle and throttle values.
  * MPC pre-processing:
    + Transform waypoints into vehicle coordinates. The coordinate frame is zeroed at vehicle location, with x-axis aligned to vehicle orientation.
    + Best-fit a cubic polynomial to vehicle-relative waypoints, using standard polynomial regression. The resulting polynomial is used as the reference (desired) trajectory for the vehicle.
    + Transform vehicle state to vehicle coordinates. This is easy--vehicle position and orientation are simply set to zero!
    + Compensate vehicle state for actuator latency.
  * Apply MPC optimization to compute steering angle & throttle:
    + Compute initial cross-track error and orientation error.
    + Setup and solve MPC optimization problem.
  * Return optimized 1st time-step actuator values to simulator.
    
## Implementation Details

### Choosing time-step duration, number of steps, etc:
  * An important part of solving the MPC optimization problem is choosing the time-step duration (dt), the number of time steps to solve for (N), and the total prediction time (I call this H = time to horizon). Note that since H = N*dt, this is actually a choice of 2 parameters, and the 3rd is implied.
  * There are multiple considerations to balance:
    - Sufficient time to horizon is necessary for MPC to effectively model the relationship between actuator changes and future state.
    - More time steps mean more computation time to solve the MPC optimization problem. Since the algorithm must run quickly, the number of time steps must be limited to a number appropriate to available compute resources.
    - At first I assumed that shorter time-step duration would contribute to more precise control, but in practice it was important to choose the other 2 parameters (and let time-step duration be determined dt = H/N).
  * I found that allowing 2 seconds for time to horizon is an effective setting at every reference velocity I tested (30-100 MPH).
    - At slower speeds (30-40 MPH) this could be decreased to 1 second, but as the speed increased it was necessary to increase this parameter.
    - I didn't find much/any advantage in decreasing time to horizon, so I just left it at a fixed 2 seconds.
  * I found that running the optimization solver over 20 time steps results in a reasonable compute load on my development PC.
    - I did test at other values like 10 or 15 time steps. At higher speeds the controller worked better with more time steps, so I left it at 20.
    - In a real-world setting I think it would be important to calibrate compute performance and adjust this parameter accordingly.
  * Because I explicitly selected the other 2 values, the time-step duration for my algorithm is set to 0.1 seconds (2 sec / 20 steps).

### MPC Optimizer cost function & parameter tuning:

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

Choosing and tuning the cost function weight parameters was a critical aspect of achieving good algorithm performance.
  * I found it was effective to use the same weight for cross-track and orientation error terms, and set the velocity term weight to half the cross-track weight.
    - This does seem to keep vehicle velocity well below reference velocity--it tops out around 75% of reference.
    - I believe with further tuning of weights we could increase the velocity term weight and maintain control. I just ran out of time for the project and left this at a reasonably functional value.
  * Weights for throttle values and rate of throttle changes are both set to around 15-20x the cross-track weight. This setting was effective at every velocity I tested.
  * Algorithm performance is highly sensitive to the weights for steering value and rate of steering change.
    - These weights are several orders of magnitude larger than cross-track error weight.
    - It worked very well to always set the rate of steering change weight to exactly half the steering value weight.
    - Good values for these 2 weight parameters seem to be highly dependent on the vehicle velocity. I found that they need to be scaled up (non-linearly) with increased velocity.
    - I wanted my algorithm to be effective for different reference velocities. My solution is as follows:
      + I manually tuned the steering value weight at 4 different speeds (30, 45, 60, 75 MPH).
      + In the final algorithm I fit a polynomial model function to these (speed, weight) samples.
      + Use this model to re-compute the steering value weight on every iteration, given current vehicle velocity.
      + The rate of steering change weight is always half the computed steering value weight.

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

### Other observations/reflections:

My main reservation with the techniques used in this project are around the use of simple cubic polynomials to model the reference trajectories.
  * The explicit y=f(x) polynomial approximation is adequate as long as the trajectory remains relatively parallel to the vehicle orientation.
  * However, problems occur if/when the trajectory diverges from vehicle orientation. This happens when either the trajectory has a sharp turn, or the vehicle (wrongly) veers away from reference trajectory. In these cases the "slope" of the cubic polynomial becomes steep, leading to numerical instability (or undefined behavior!) in the explicit y=f(x) function.
  * A much better solution for the trajectory would be to compute a parametric curve (B-spline, Bezier spline, etc.) or non-parametric curve (smoothing spline, thin-plate spline). These are designed to model 2D curves without the difficulties of using explicit functions. Of course, they're also more complex mathematically and were definitely out-of-scope for completing this project on time.

Overall, I was very impressed with the effectiveness of the MPC approach compared with the PID controller approach we implemented for the previous project. I was able to safely "drive" my vehicle at 3x the speed with much less time spent tuning the controller.


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
