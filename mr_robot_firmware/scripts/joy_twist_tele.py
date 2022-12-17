#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from termcolor import colored

colors = ['red', 'green', 'cyan', 'blue', 'magenta']
class TwistJoy:
    def __init__(self):        
        rospy.init_node('Joy2Turtle')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    
        rospy.Subscriber("/joy", Joy, self.callback)    
        self.axes=0
        self.buttons=0
        self.aL=0
        self.aR=0
        self.L1 = 0
        self.R1 = 0
        self.start()


    def callback(self,data):    
        self.axes = data.axes
        self.buttons = data.buttons
        self.aL = self.axes[1]
        self.aR = self.axes[0]
        self.R1 = self.buttons[5]
        self.L1 = self.buttons[4]


    def start(self):
        rate = rospy.Rate(15)
        twist = Twist()
        while not rospy.is_shutdown():
            twist.linear.x = 1.15*self.aL
            twist.angular.z = 2*self.aR
            if self.L1 == 1:    
                twist.angular.z = 2*self.L1
            elif self.R1 == 1:    
                twist.angular.z = -2*self.R1 
            self.pub.publish(twist)
            rate.sleep()
