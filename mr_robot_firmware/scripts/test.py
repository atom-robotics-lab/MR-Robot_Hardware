#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist



rospy.init_node('cmdvel_publisher', anonymous=False)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
velocity_msg = Twist(

while True :
    velocity_msg.linear.x = 1
    velocity_msg.angular.z = 0
    pub.publish(self.velocity_msg)