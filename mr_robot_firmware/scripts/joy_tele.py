#!/usr/bin/env python
from matplotlib.pyplot import axes
from matplotlib.widgets import Button
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy



 


def callback(data):    
    twist = Twist()    
    twist.linear.x = 1.15*data.axes[1]    
    twist.angular.z = 1.15*data.axes[0]
    if data.buttons[4] == 1:    
        twist.angular.z = 1.15*data.buttons[4]
    elif data.buttons[5] == 1:    
        twist.angular.z = -1.15*data.buttons[5]

    
    print(data)
    pub.publish(twist)


def start():    
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist)    
    rospy.Subscriber("joy", Joy, callback)    
    rospy.init_node('Joy2Turtle')
    rospy.spin()

if __name__ == '__main__':
    start()
