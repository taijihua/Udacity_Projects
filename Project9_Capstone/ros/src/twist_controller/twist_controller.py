from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # Implement
        if args: # if args is NOT empty
            pass
        if kwargs: # if kwargs is NOT empty
            self.wheel_base = kwargs['wheel_base']
            self.steer_ratio = kwargs['steer_ratio']
            self.min_speed = kwargs['min_speed']
            self.max_lat_accel = kwargs['max_lat_accel']
            self.max_steer_angle = kwargs['max_steer_angle']
            self.vehicle_mass = kwargs['vehicle_mass']
            self.wheel_radius = kwargs['wheel_radius']
            self.decel_limit = kwargs['decel_limit']

        # PID controller for throttle, udacity video values are 0.3, 0.1, 0.0, 0.0, 0.2
        self.linearController = PID(kp=0.3, ki=0.1, kd=0., mn=0.0, mx=0.3)
        #self.linearController = PID(kp=0.3, ki=0.05, kd=0., mn=0.0, mx=0.6)

        self.last_vel = None  #store last linear velocity value
        self.last_time = rospy.get_time()  #store last time for getting elapsed time for each step (for integral purpose)
        # Yaw controller
        #self.yawController = YawController(self.wheel_base, self.steer_ratio, 0.1, self.max_lat_accel, self.max_steer_angle)
        self.yawController = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)
        tau = 0.5  # 1/(2pi*tau) = cutoff frequency
        #tau = 0.5  # 1/(2pi*tau) = cutoff frequency
        ts = 0.02  # sample time
        self.vel_lpf = LowPassFilter(tau, ts) #low pass filter for noisy velocity data
        pass

    def control(self, *args, **kwargs):
        # Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if args:
            is_dbw_enabled, cmd_linear_vel, cmd_angular_vel, current_linear_vel = args
        if kwargs: # if kwargs is NOT empty
            pass
        if is_dbw_enabled is False:
            self.linearController.reset()
            return 0, 0, 0
        throttle = 0. #1.0
        brake = 0
        steering = 0
        current_linear_vel = self.vel_lpf.filt(current_linear_vel)
        steering = self.yawController.get_steering(cmd_linear_vel, cmd_angular_vel, current_linear_vel)

        ## Calculate throttle/brake
        vel_error = cmd_linear_vel - current_linear_vel
        self.last_vel = current_linear_vel
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        throttle = self.linearController.step(vel_error, sample_time)
        
        #rospy.logwarn('vel_error = %f', vel_error)
        

        if cmd_linear_vel==0 and current_linear_vel<.1: #0.1 #hold vehicle in place
            throttle = 0
            brake = 700 # N*m, to hold the car in place if vehicle is stopped. Acc ~ 1m/s^2
        #elif throttle < .1 and vel_error<0:  # deceleration  
        elif throttle < .1 and vel_error<0:  # deceleration  
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius
        rospy.logwarn('cmd_vel= %f, curr_vel= %f throttle= %f, brake= %f, steering= %f', cmd_linear_vel, current_linear_vel, throttle, brake, steering)

        return throttle, brake, steering
