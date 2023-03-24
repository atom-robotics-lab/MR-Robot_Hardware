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
        self.ak_up_down=0
        self.ak_left_right=0
        self.joyL = 0
        self.R2 = 0
        self.L2 = 0
        self.max_linear = 1
        self.max_angular = 1
        self.increament = 0.1
        self.motor_control()


    def callback(self,data):    
        self.axes = data.axes
        self.buttons = data.buttons
        self.ak_up_down = self.axes[7]
        self.ak_left_right = self.axes[6]
        self.joyL = self.axes[0]
        self.R2 = self.axes[5]
        self.L2 = self.axes[2]
        self.motor_control()
        
    def map_range(self, x, in_min, in_max, out_min, out_max):
      return ((x - in_min) * (out_max - out_min) /(in_max - in_min)) + out_min


    def motor_control(self):
        forward = self.map_range(-1*self.R2, -1, 1, 0, 1)
        backward = self.map_range(-1*self.L2, -1, 1, 0, 1)
        #print("forward: {}  backward: {}".format(forward, backward))
        self.vel.linear.x = (forward * self.max_linear) - (backward * self.max_linear)
        self.vel.angular.z = self.joyL * self.max_angular * 1 

        if self.ak_up_down == 1.0:
            self.max_linear = self.max_linear + self.increament
        
        if self.ak_up_down == -1.0:
            self.max_linear = self.max_linear - self.increament
        
        if self.ak_left_right == 1.0:
            self.max_angular = self.max_angular + self.increament
        
        if self.ak_left_right == -1.0:
            self.max_angular = self.max_angular - self.increament

        print("Linear: {}   Angular: {}     max_linear: {}      min_linear: {}".format(self.vel.linear.x, self.vel.angular.z,   self.max_linear,    self.max_angular))

        self.pub.publish(self.vel)
        


if __name__ == "__main__":
    TJ = TwistJoy()
    rospy.spin()