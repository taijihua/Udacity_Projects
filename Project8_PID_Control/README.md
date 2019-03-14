# CarND-PID-Control-Project
Project submit for Self-Driving Car Engineer Nanodegree Program, the following are project rubic and my response
   
### Goals
* #### Your code should compile.
    My code was able to compile sucessfully with cmake and make
    
* #### The PID procedure follows what was taught in the lessons.
    The PID procedure follows what was taught in the lessons, at each iteration three errors (p_error, i_error and d_error) were updated, and the control output was calculated based on the equation -p_error*Kp-i_error*Ki-d_error*Kd
    
* #### Describe the effect each of the P, I, D components had in your implementation.
    The P component respond to the difference between the measurement and target, the I component responds to the integral error between the measurement and target, while the D component responds to the changes in p errors over time.
    
* #### Describe how the final hyperparameters were chosen.
    A twiddle search algorithm was implemented.
    
* #### The vehicle must successfully drive a lap around the track.
    Yes the vehicle was able to successfully drive a lap around the track.
