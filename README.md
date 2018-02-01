# Model Predictive Control
Self-Driving Car Engineer Nanodegree Program

A Model Predictive Control (MPC) is implemented in C++ to control a vehicle along a track.
At each time step, the simulator provides the waypoints that the vehicle is supposed to follow.
The points are fitted to a polynomial, and a future trajectory is calculated by minimizing an error function.
The effect of latency is considered, and the control parameters are sent back to the simulator.
The vehicle is modeled by six paraterers that include cross track error and the error in psi, and is controled by the steering and actuation.

---
[imageRMS]: rms.png "RMS"

## Vehicle Model

The state of the vehicle is modeled by six parameters; x position (`x`), y position (`y`), yaw angle (`psi`), speed (`v`), cross track error (`CTE`), and the error in psi (`epsi`).
The state variables evolves in time step dt as following (MPC.cpp line 41):

- x(t+1) = x(t) + v(t) * cos(psi(t)) * dt
- y(t+1) = y(t) + v(t) * sin(psi(t)) * dt
- psi(t+1) = psi(t) + v(t) * delta(t) * / Lf * dt
- v(t+1) = v(t) + a(t) * dt
- cte(t+1) = f(x(t)) - y(t) + v(t) * sin(epsi(t)) * dt
- epsi(t+1) = psi(t) - psides(t) + v(t) * delta(t) / Lf * dt

`Lf` is the distance between the center of mass of the vehicle and the fron axle.
The steering angle `delta` is limited within +/- 25 degrees.

## Timestep

The timestep `dt` for the trajectory calculation is set to 0.1 second.
If `dt` is too large, the calculation error would increase. If it was too short, the trajectory calculation may take too long.
Assuming the speed of 30 mph, a vehicle will proceed about 5 meters in 0.1 second.
That is about the length of a vehicle, and if the trajectory is sufficiently stable, one can assume that the motion of the vehicle would not be too complicated to cause an excessive error in the calculation while moving that distance.

The number of timesteps `N` is set to 15. The number should be large enough for a predictive control to take the relevant future information into account.
However, it it was too long, resources may be wasted for unnecessary calculations.
I chose the number so that the predicted route, drawn in a green line in the simulator, covers the whole range of the steepest curve in the simulator.

## Fitting on Waypoints

The simulator provides a set of x and y corrdinates along the center of the track.
The points are fitted to a third degree polynomial which is used to guide the vehicle.
In order to be used for visualization, the points are transformed to the vehicle coordinate system from the map coordinate system by the function `toCarCoord()` (Tools.cpp line 68).
In the simulator, the fitted line is drawn in yellow.

## Trajectory Optimization

The trajectory is calculated by minimizing the cost function as defined in `FG_eval::operator()`.

```c++
    // Reference State Cost
    for (int t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2) * extra_weight;
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    for (int t = 0; t < N - 1; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t], 2) * extra_weight;
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }
    for (int t = 0; t < N - 2; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2) * extra_weight;
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```

First three terms are to minimize the squared sum of CTE, epsi, and the speed difference from the reference speed.
Next two terms control the magnitudes of steering angle and actuation.
Final two tems are intended to avoid aprupt changes in steering and actuation.
In an early attempt with equal weights on each terms, the simulation showed a lot of zigzagging in the trajectory.
Therfore I applied `extra_weight` on the terms that regulate the errors that are calculated from angular components; epsi, and steering angle.
In order to decide on the value of the extra weight, I measured RMS of CTE, epsi, steering, and the steerig increment at each timestep, with the weigt set to different values.
The test is done with the first 400 calculations and `ref_v` set to 30.
The result is shown in the next plot.
As the extra weight increases, the RMS of epsi and steering decrease, but the RMS of cte increases.
and I chose 100 that seems to be a good compromise between two tendencies.

![alt_text][imageRMS]

## Latency

The vehicle control takes its effect with some latency.
In the simulation, I set the latency to 100 ms, and implemented its effect in the function `applyLatency()` (Tools.cpp, line 60).
The function updates the values `x`, `y`, and `psi` as following.

- x = x + v * dt
- y = y + v * dt
- psi = psi - v * delta / Lf * dt

The RMS values with and without the latency correction shows that there is a significant difference.
Following result is calculated with the same parameter settings as in the test that decided extra weight in above section.

|    | cte | epsi | steer | dsteer |
|:--:|:---:|:----:|:-----:|:------:|
| with correction    | 0.2400 | 0.0207 | 0.0299 | 0.0126 |
| without correction | 0.2951 | 0.0527 | 0.0380 | 0.0180 |











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



