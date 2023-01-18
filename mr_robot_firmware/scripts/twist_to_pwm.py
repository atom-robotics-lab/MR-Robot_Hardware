#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time
from math import pi



class DifferentialDriver :
    def __init__(self) :

        rospy.init_node('cmdvel_listener', anonymous=False)
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        rospy.Subscriber("left_speed", Twist, self.update_left)
        rospy.Subscriber("right_speed", Twist, self.update_right)

        self.left_pwm_pub = rospy.Publisher('left_pwm', Int32, queue_size=10)
        self.right_pwm_pub = rospy.Publisher('right_pwm', Int32, queue_size=10)

        self.left_pwm = Int32()
        self.right_pwm = Int32()

        self.params_setup() 
        self.wheel_radius = self.wheel_diameter/2
        self.circumference_of_wheel = 2 * pi * self.wheel_radius
        self.max_speed = (self.circumference_of_wheel*self.motor_rpm)/60   # m/sec
        self.left_vel = 0
        self.right _vel = 0


    def params_setup(self) :
        
        self.motor_rpm = rospy.get_param("mr_robot_firmware/motor_rpm")
        self.wheel_diameter = rospy.get_param("mr_robot_firmware/wheel_diameter")
        self.wheel_diameter = self.wheel_diameter/100
        self.wheel_separation = rospy.get_param("mr_robot_firmware/wheel_diameter")
        self.wheel_separation = self.wheel_separation/100
        self.max_pwm_val = rospy.get_param("mr_robot_firmware/twist_max_pwm")
        self.min_pwm_val = rospy.get_param("mr_robot_firmware/twist_min_pwm")

    def update_left(self, speed):
        self.left_vel_actual = speed.data

    def update_right(self, speed):
        self.right_vel_actual = speed.data
        
          

    def stop( self ):
        self.change_duty_cycle(0, 0 , 0 , 0)       
        
    def get_pwm(self, left_speed, right_speed):
        
        lspeedPWM = max(min((left_speed/self.max_speed)*self.max_pwm_val, self.max_pwm_val), self.min_pwm_val)
        rspeedPWM = max(min((right_speed/self.max_speed)*self.max_pwm_val,self.max_pwm_val), self.min_pwm_val)

        return lspeedPWM, rspeedPWM
    
    def callback(self, data):  

        linear_vel = data.linear.x                                              # Linear Velocity of Robot
        angular_vel = data.angular.z                                            # Angular Velocity of Robot

        right_vel = linear_vel + (angular_vel * self.wheel_separation) / 2      # right wheel velocity
        left_vel  = linear_vel - (angular_vel * self.wheel_separation) / 2      # left wheel velocity

        print(" Left Velocity = {}  |   Right Velocity  |   Left Actual = {}    |   Right Actual = {}".format(left_vel, left_vel, self.right_vel_actual, self.right_vel_actual))
        
        left_pwm_data , right_pwm_data = self.get_pwm(left_vel, right_vel)

        #print(left_pwm_data) 
        #print(right_pwm_data) 


        self.left_pwm.data = int(left_pwm_data)
        self.right_pwm.data = int(right_pwm_data)


        self.left_pwm_pub.publish(self.left_pwm)
        self.right_pwm_pub.publish(self.right_pwm)




if __name__== '__main__':

    dd = DifferentialDriver()    
    
    rospy.loginfo('Differential Drive Initialized with following Params-')
    rospy.loginfo('Motor Max RPM:\t'+str(dd.motor_rpm)+' RPM')
    rospy.loginfo('Wheel Diameter:\t'+str(dd.wheel_diameter)+' m')
    rospy.loginfo('Wheel Separation:\t'+str(dd.wheel_separation)+' m')
    rospy.loginfo('Robot Max Speed:\t'+str(dd.max_speed)+' m/sec')
    rospy.loginfo('Robot Max PWM:\t'+str(dd.max_pwm_val))

    rospy.spin()
