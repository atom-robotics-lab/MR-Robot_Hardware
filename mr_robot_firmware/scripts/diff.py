#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time
from math import pi


GPIO.setmode(GPIO.BCM)

GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)


class DifferntialDriver :
    def __init__(self) :
        self.motor_rpm = rospy.get_param("mr_robot_firmware/motor_rpm")
        self.wheel_diameter = rospy.get_param("mr_robot_firmware/wheel_diameter")
        self.wheel_separation = rospy.get_param("mr_robot_firmware/wheel_seperation")
        self.max_pwm_val = rospy.get_param("mr_robot_firmware/max_pwm_val")
        self.min_pwm_val = rospy.get_param("mr_robot_firmware/min_pwm_val")

        self.wheel_radius = self.wheel_diameter/2
        self.circumference_of_wheel = 2 * pi * self.wheel_radius
        self.max_speed = (self.circumference_of_wheel*self.motor_rpm)/60   #   m/sec

        self.lapwm = GPIO.PWM(5, 1000)
        self.lbpwm = GPIO.PWM(6, 1000)
        self.rapwm = GPIO.PWM(16, 1000)
        self.rbpwm = GPIO.PWM(20, 1000)

        self.lapwm.start(0)
        self.lbpwm.start(0)
        self.rapwm.start(0)
        self.rbpwm.start(0)




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

        try :

            linear_vel = data.linear.x                                              # Linear Velocity of Robot
            angular_vel = data.angular.z                                            # Angular Velocity of Robot

            print('linear and angular: ', linear_vel, angular_vel)
            right_vel = linear_vel + (angular_vel * self.wheel_separation) / 2      # right wheel velocity
            left_vel  = linear_vel - (angular_vel * self.wheel_separation) / 2      # left wheel velocity
            print('left speed and right speed: ', left_vel, right_vel)

            l_pwm, r_pwm = self.get_pwm(left_vel, right_vel)

            print('left pwm and right pwm: ', l_pwm, r_pwm)

            if (left_vel == 0.0 and right_vel == 0.0):
                self.stop()
                print("stopping")

            elif (left_vel >= 0.0 and right_vel >= 0.0):
                self.lapwm.ChangeDutyCycle(l_pwm)
                self.lbpwm.ChangeDutyCycle(0)
                self.rapwm.ChangeDutyCycle(r_pwm)
                self.rbpwm.ChangeDutyCycle(0)
                print("moving forward")

            elif (left_vel <= 0.0 and right_vel <= 0.0):

                self.lapwm.ChangeDutyCycle(0)
                self.lbpwm.ChangeDutyCycle(l_pwm)
                self.rapwm.ChangeDutyCycle(0)
                self.rbpwm.ChangeDutyCycle(r_pwm)
                print("moving backward")

            elif (left_vel < 0.0 and right_vel > 0.0):
                self.lapwm.ChangeDutyCycle(0)
                self.lbpwm.ChangeDutyCycle(l_pwm)
                self.rapwm.ChangeDutyCycle(r_pwm)
                self.rbpwm.ChangeDutyCycle(0)
                print("turning left")

            elif (left_vel > 0.0 and right_vel < 0.0):
                self.lapwm.ChangeDutyCycle(l_pwm)
                self.lbpwm.ChangeDutyCycle(0)
                self.rapwm.ChangeDutyCycle(0)
                self.rbpwm.ChangeDutyCycle(r_pwm)
                print("turning right")

            else:
                self.stop()

        except KeyboardInterrupt:
            print("Keyboard Interruption")

        except :
            print("Other error or exception occured")

        finally :
            print("Cleaning GPIO")
            GPIO.cleanup()

        
    def listener( self ):
        rospy.init_node('cmdvel_listener', anonymous=False)
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        rospy.spin()




if __name__== '__main__':
    dd = DifferntialDriver()    
    
    print('Differntial Drive Initialized with following Params-')
    print('Motor Max RPM:\t'+str(dd.motor_rpm)+' RPM')
    print('Wheel Diameter:\t'+str(dd.wheel_diameter)+' m')
    print('Wheel Separation:\t'+str(dd.wheel_separation)+' m')
    print('Robot Max Speed:\t'+str(dd.max_speed)+' m/sec')

    dd.listener()