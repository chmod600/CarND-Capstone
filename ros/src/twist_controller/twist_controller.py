#!/usr/bin/env python
# By Richard Lee

import rospy

from yaw_controller import YawController
import pid
import lowpass

MAX_TORQUE = 350.0 

class Controller(object):			

    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
	
	kp = 0.45
	ki = 0.003
	kd = 0.25
	mn = -max_steer_angle
	mx = max_steer_angle
	tau = 0.5
	ts = 0.02
	self.vel_lpf = lowpass.LowPassFilter(tau, ts)
 
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.0, max_lat_accel, max_steer_angle)

        self.steering_correction_pid = pid.PID(kp, ki, kd, mn, mx)

        self.timestamp = rospy.get_time()

    def reset(self):
        self.steering_correction_pid.reset()


    # For Steering Control
    def control(self, cte, dbw_enabled, linear_vel, angular_vel, current_lin_vel):		

        new_timestamp = rospy.get_time()

        duration = new_timestamp - self.timestamp

        self.timestamp = new_timestamp

        if dbw_enabled:  
            
            target_steering_angle = self.steering_correction_pid.step(cte, duration)

            yaw_steer = self.yaw_controller.get_steering(linear_vel, angular_vel, current_lin_vel)
	    yaw_steer = self.vel_lpf.filt(yaw_steer)

            steering_angle = target_steering_angle + yaw_steer

            return steering_angle

        else: 
 
            self.steering_correction_pid.reset()  

        return 0.0


    # For speed control
    def control_speed(self, target_linear_velocity, current_velocity, max_throttle_proportional, max_brake_proportional):

        velocity_change_required = target_linear_velocity - current_velocity
        throttle, brake = 0, 0

        if velocity_change_required > 0.1:
            
            throttle, brake = min(3.0 * velocity_change_required / target_linear_velocity, max_throttle_proportional), 0.0
            
        elif velocity_change_required < -0.1:
            
            throttle, brake = 0.0, min(3.0 * velocity_change_required / target_linear_velocity, max_brake_proportional)
            
	    return throttle, brake
