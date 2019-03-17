# CarND-PID-Control-Project
This Project is submitted for Self-Driving Car Engineer Nanodegree Program, the following are project rubic and my response
   
### Goals
* #### Your code should compile.
    My code was able to compile sucessfully with cmake and make
    
* #### The PID procedure follows what was taught in the lessons.
    The PID procedure follows what was taught in the lessons, at each iteration three errors (p_error, i_error and d_error) were updated (function PID::UpdateError() in pid.cpp), and the control output was calculated based on the equation -p_error\*Kp-i_error\*Ki-d_error\*Kd (function PID::CalcOutput() in pid.cpp)
    
* #### Describe the effect each of the P, I, D components had in your implementation.
    The P component respond to the difference between the measurement and target, the I component responds to the integral error between the measurement and target, while the D component responds to the changes in p errors over time. In my particular PID controller for the steering, the final optimized I coefficient is zero which has no effect, the P coefficient drives the steering wheel towards the center whenever the vehicle deviates from center line, and the D coefficient has greatest value of the three and it works to dynamically adjust the steering input based on if the deviation from center is getting larger or smaller.
    
* #### Describe how the final hyperparameters were chosen.
    A twiddle search algorithm was implemented (line 49-70 and line 128-181 in main.cpp). The search started with Kp, Ki, and Kd of 0, 0, 0 and modification step of 1 for each parameter (stored in , then based on the result from modified PID parameters the twiddle algorithm change the step size (reduce step size by 10% if results are not getting better, or increase by 10% if results are better). The search stopped when the overall step sizes become less than a threshold (0.001 in my code).
    I also implemented a PID controller for speed control with throttle value output, the same twiddle search were also applied to speed PID controller (line 52 and 53 in main.cpp was used for controlling if twiddle search was applied to steering PID controller or throttle PID controller).
    During my tuning, i first fixed throttle value and searched for best parameters for steering PID controller, and then fixed the steering PID controller parameter and search for a best throttle PID controller. The final parameters can be found in line 45 and 47 in main.cpp.
    
* #### The vehicle must successfully drive a lap around the track.
    Yes the vehicle was able to successfully drive a lap around the track, at speed of 25. Somehow i wasn't able to find a good paraemter value set for higher speed, and the driving would become very zig-zag if speed increases (by changing the last parameter value in line 47 pid_speed.Init() function)... May need a different hyperparameter search algorithm than twiddle.
