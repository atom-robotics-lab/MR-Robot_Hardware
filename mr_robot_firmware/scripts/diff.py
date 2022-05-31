#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time
from math import pi



class DifferentialDriver :
    def __init__(self) :

        self.params_setup() 
        self.gpio_setup()  


        self.wheel_radius = self.wheel_diameter/2
        self.circumference_of_wheel = 2 * pi * self.wheel_radius
        self.max_speed = (self.circumference_of_wheel*self.motor_rpm)/60   # m/sec

        
    def gpio_setup(self) :

        self.GPIO = GPIO

        self.GPIO.setmode(GPIO.BCM)
        self.GPIO.setup(self.high1 , GPIO.OUT)
        self.GPIO.setup(self.ground1 , GPIO.OUT)
        self.GPIO.setup(self.high2 , GPIO.OUT)
        self.GPIO.setup(self.ground2 , GPIO.OUT)

        self.lapwm = self.GPIO.PWM(self.high1 , 1000)
        self.lbpwm = self.GPIO.PWM(self.ground1 , 1000)
        self.rapwm = self.GPIO.PWM(self.high2 , 1000)
        self.rbpwm = self.GPIO.PWM(self.ground2 , 1000)

        self.lapwm.start(0)
        self.lbpwm.start(0)
        self.rapwm.start(0)
        self.rbpwm.start(0)


    def params_setup(self) :
        
        self.motor_rpm = rospy.get_param("mr_robot_firmware/motor_rpm")
        self.wheel_diameter = rospy.get_param("mr_robot_firmware/wheel_diameter")
        self.wheel_separation = rospy.get_param("mr_robot_firmware/wheel_seperation")
        self.max_pwm_val = rospy.get_param("mr_robot_firmware/max_pwm_val")
        self.min_pwm_val = rospy.get_param("mr_robot_firmware/min_pwm_val")
        
        self.high1 = rospy.get_param("mr_robot_firmware/high1")
        self.ground1 = rospy.get_param("mr_robot_firmware/ground1")
        self.high2 = rospy.get_param("mr_robot_firmware/high2")
        self.ground2 = rospy.get_param("mr_robot_firmware/ground2")   

    def stop( self ):

        self.lapwm.ChangeDutyCycle(0)
        self.lbpwm.ChangeDutyCycle(0)
        self.rapwm.ChangeDutyCycle(0)
        self.rbpwm.ChangeDutyCycle(0)
        

    def get_pwm(self, left_speed, right_speed):
        
        lspeedPWM = min((left_speed/self.max_speed)*self.max_pwm_val, self.max_pwm_val)
        rspeedPWM = min((right_speed/self.max_speed)*self.max_pwm_val, self.max_pwm_val)

        return abs(lspeedPWM), abs(rspeedPWM)
    
    def callback(self, data):  

        linear_vel = data.linear.x                                              # Linear Velocity of Robot
        angular_vel = data.angular.z                                            # Angular Velocity of Robot
        rospy.loginfo('linear and angular: ', linear_vel, angular_vel)

        right_vel = linear_vel + (angular_vel * self.wheel_separation) / 2      # right wheel velocity
        left_vel  = linear_vel - (angular_vel * self.wheel_separation) / 2      # left wheel velocity
        rospy.loginfo('left speed and right speed: ', left_vel, right_vel)
        
        l_pwm, r_pwm = self.get_pwm(left_vel, right_vel)
        rospy.loginfo('left pwm and right pwm: ', l_pwm, r_pwm)
        
        if (left_vel == 0.0 and right_vel == 0.0):
            self.stop()
            rospy.loginfo("stopping")

        elif (left_vel >= 0.0 and right_vel >= 0.0):
            self.lapwm.ChangeDutyCycle(l_pwm)
            self.lbpwm.ChangeDutyCycle(0)
            self.rapwm.ChangeDutyCycle(r_pwm)
            self.rbpwm.ChangeDutyCycle(0)
            rospy.loginfo("moving forward")
        
        elif (left_vel <= 0.0 and right_vel <= 0.0):
            self.lapwm.ChangeDutyCycle(0)
            self.lbpwm.ChangeDutyCycle(l_pwm)
            self.rapwm.ChangeDutyCycle(0)
            self.rbpwm.ChangeDutyCycle(r_pwm)
            rospy.loginfo("moving backward")
        
        elif (left_vel < 0.0 and right_vel > 0.0):
            self.lapwm.ChangeDutyCycle(0)
            self.lbpwm.ChangeDutyCycle(l_pwm)
            self.rapwm.ChangeDutyCycle(r_pwm)
            self.rbpwm.ChangeDutyCycle(0)
            rospy.loginfo("turning left")
        
        elif (left_vel > 0.0 and right_vel < 0.0):
            self.lapwm.ChangeDutyCycle(l_pwm)
            self.lbpwm.ChangeDutyCycle(0)
            self.rapwm.ChangeDutyCycle(0)
            self.rbpwm.ChangeDutyCycle(r_pwm)
            rospy.loginfo("turning right")
        
        else:
            self.stop()

        
        
    def cmd_callback( self ):
        rospy.init_node('cmdvel_listener', anonymous=False)
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        rospy.spin()

    def gpio_cleanup(self) :
        self.GPIO.cleanup()




if __name__== '__main__':

    dd = DifferentialDriver()    
    
    rospy.loginfo('Differntial Drive Initialized with following Params-')
    rospy.loginfo('Motor Max RPM:\t'+str(dd.motor_rpm)+' RPM')
    rospy.loginfo('Wheel Diameter:\t'+str(dd.wheel_diameter)+' m')
    rospy.loginfo('Wheel Separation:\t'+str(dd.wheel_separation)+' m')
    rospy.loginfo('Robot Max Speed:\t'+str(dd.max_speed)+' m/sec')

    
    try :
        dd.cmd_callback()
    
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard Interruption")

    except :
        rospy.loginfo("Other error or exception occured")

    finally :
        rospy.loginfo("Cleaning GPIO")
        dd.gpio_cleanup()

    
