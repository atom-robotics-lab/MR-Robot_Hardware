#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


colors = ['red', 'green', 'cyan', 'blue', 'magenta']

class TwistJoy:
    def __init__(self):        
        rospy.init_node('Joy2Twist')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    
        rospy.Subscriber("/joy", Joy, self.callback)    
        self.vel = Twist()
        self.axes=0
        self.buttons=0
        self.aL=0
        self.aR=0
        self.L1 = 0
        self.R1 = 0
        self.fac1 = 2
        self.fac2 = 10
        self.up = 0
        self.down = 0
        self.left = 0
        self.right = 0
        self.max_speed = 10
        self.max_linear = 1
        self.max_angular = 1
        self.increament = 0.5
        self.motor_control()


    def callback(self,data):    
        self.axes = data.axes
        self.buttons = data.buttons
        self.ak_up_down = self.buttons[7]
        self.ak_left_right = self.buttons[6]
        self.joyL = self.axes[0]
        self.R2 = self.axes[6]
        self.L2 = self.axes[2]
        self.motor_control()
        


    def motor_control(self):
        self.vel.linear.x = abs((0.5*(self.R2+1)-1)) * self.max_linear - abs((0.5*(self.L1+1)-1)) * self.max_linear
        self.vel.angular.z = self.joyL * self.max_angular

        if self.ak_up_down == 1.0:
            self.max_linear = self.max_linear + self.increament
        
        if self.ak_up_down == -1.0:
            self.max_linear = self.max_linear - self.increament
        
        if self.ak_left_right == 1.0:
            self.max_angular = self.max_linear + self.increament
        
        if self.ak_left_right == -1.0:
            self.max_angular = self.max_linear - self.increament

        print("Linear: " + self.vel.linear.x)
        print("Angular: " + self.vel.angular.z)

        self.vel.publish()
        


if __name__ == "__main__":
    TJ = TwistJoy()
    rospy.spin()