#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
from math import pi

motor_rpm = 60              #   max rpm of motor on full voltage 
wheel_diameter = 0.065      #   in meters
wheel_separation = 0.17     #   in meters
max_pwm_val = 100           #   100 for Raspberry Pi , 255 for Arduino
min_pwm_val = 30            #   Minimum PWM value that is needed for the robot to move

wheel_radius = wheel_diameter/2
circumference_of_wheel = 2 * pi * wheel_radius
max_speed = (circumference_of_wheel*motor_rpm)/60   #   m/sec
#12.246

def stop():
    #print('stopping')
    lspeedPWM = 0
    rspeedPWM = 0
    return lspeedPWM, rspeedPWM

def refine(left_speed, right_speed):
    global max_pwm_val
    global min_pwm_val
    lspeedPWM = (left_speed/max_speed)*max_pwm_val
    rspeedPWM = (right_speed/max_speed)*max_pwm_val

    return lspeedPWM, rspeedPWM
    
def callback(data):
    global wheel_radius
    global wheel_separation
    print("hi")
    linear_vel = data.linear.x                  # Linear Velocity of Robot
    angular_vel = data.angular.z                # Angular Velocity of Robot
    
    print('linear and angular: ', linear_vel, angular_vel)
    right_vel = linear_vel + (angular_vel * wheel_separation) / 2       # right wheel velocity
    left_vel  = linear_vel - (angular_vel * wheel_separation) / 2              # left wheel velocity
    
    if (left_vel == 0.0 and right_vel == 0.0):
        stop()
        print("stopping")

    elif (left_vel >= 0.0 and right_vel >= 0.0):
        print(refine(abs(left_vel), abs(right_vel)))
        print("moving forward")

    elif (left_vel <= 0.0 and right_vel <= 0.0):
        print(refine(abs(left_vel), abs(right_vel)))
        print("moving backward")

    elif (left_vel < 0.0 and right_vel > 0.0):
        print(refine(abs(left_vel), abs(right_vel)))
        print("turning left")

    elif (left_vel > 0.0 and right_vel < 0.0):
        print(refine(abs(left_vel), abs(right_vel)))
        print("turning right")
        
    else:
        stop()
        
def listener():
    rospy.init_node('cmdvel_listener', anonymous=False)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__== '__main__':
    print('Tortoisebot Differential Drive Initialized with following Params-')
    print('Motor Max RPM:\t'+str(motor_rpm)+' RPM')
    print('Wheel Diameter:\t'+str(wheel_diameter)+' m')
    print('Wheel Separation:\t'+str(wheel_separation)+' m')
    print('Robot Max Speed:\t'+str(max_speed)+' m/sec')
    listener()