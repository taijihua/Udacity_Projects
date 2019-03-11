# CarND-Path-Planning-Project
Project submit for Self-Driving Car Engineer Nanodegree Program
   
### Goals
* #### The code compiles correctly.
    The code was able to compile sucessfully
* #### The car is able to drive at least 4.32 miles without incident..
    Yes the car was able to drive >4.32 miles without incident..
* #### The car drives according to the speed limit.
    The car drives according to the speed limit.
* #### Max Acceleration and Jerk are not Exceeded.
    The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3. In face, I set a global constant variable for the maximum acceleration which is set to be 8 m/s^2, and maximum jerk was satisfied with the spline function smoothing. And every iteration the vehicle keeps part of previously planned path for smooth transition, max number of history path points is configuration (currently 5 ponits).

* #### Car does not have collisions.
    The car was able to drive without collisions.

* #### The car stays in its lane, except for the time between changing lanes.
    The car only changes its lane when necessary and the time for lane change doesn't exceed 3 seconds.

* #### The car is able to change lanes

    The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

### How the path planner works

The path planner uses idea of finite state machine, and the state of the vehicle can be one of these three {*keep_lane*, *move_left*, *move_right*}.
    At each iteration, the path planner reuses previous iteration's planned path for up to 5 data points (configurable at line 124 in main.cpp), this helps with smooth transitions and minimize the jerk.
    The path planner scans through the sensor fusion data to prepare some flags for deciding to keep lane or change lane, i.e, *left_traffic* becomes true if there is object on left lane within a predefined safe distance (*safe_dist*) of current vehicle (code line 205-218 in main.cpp), similarly there is a *right_traffic* flag (code line 220-233). And *target_v* is an array that stores maximum speed the vehicle could safely travel in each lane, based on the objects in front of current vehicle in each lane.
    The path planner then decide whether to change lane left, right or stay in lane, based on the flags set in the above step. Detailed logic is below:
  - if current state is *keep_lane* 
    - if it's safe to move left (*left_traffic* is false) and the target speed can increase (*target_v[curr_lane-1]>target_v[curr_lane]*), switch state to *move_left* and set target lane to be left or right lane.
    - else if it's safe to move right (*right_traffic* is false) and the target speed can increase (*target_v[curr_lane+1]>target_v[curr_lane]*), switch state to *move_right*.
  - if current state is *move_left* 
    - if current lane is already the target lane, switch state to *keep_lane*
  - if current state is *move_right* 
    - if current lane is already the target lane, switch state to *keep_lane* 
   
After the path planner decides what to do next, it will generate the trajectory by adding a few waypoints that are 30, 60, and 90 meters ahead of current vehicle, and the d value is dependent on the target lane. These waypoints in addition to current vehicle location and previously planned path points are all used for spline fitting to get smooth trajectory. During this process the x-y coordinate is tansformed to vehicle coordinate to help fitting, and transformed back to x-y coordinates for getting path points. When picking points for path planner, the vehicle speed and acceleration is being considered to make sure the distance between points are within our requirement for max speed and acceleration (code line 365-369, line 376-383)
