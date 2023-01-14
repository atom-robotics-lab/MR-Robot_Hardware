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
        self.start()


    def callback(self,data):    
        self.axes = data.axes
        self.buttons = data.buttons
        self.aL = self.axes[1]
        self.aR = self.axes[0]
        self.R1 = self.buttons[5]
        self.L1 = self.buttons[4]
        self.up = self.axes[7]
        self.down = self.axes[7]
        self.left = self.axes[6]
        self.right = self.axes[6]
        


    def start(self):
        rate = rospy.Rate(15)
        twist = Twist()
        while not rospy.is_shutdown():
            
            if self.up == 1:
                self.fac1 += 0.05
                rospy.loginfo("Linear = %f: Angular =%f\n",self.fac1,self.fac2)

            elif self.down == -1:
                self.fac1 -= 0.05
                rospy.loginfo("Linear = %f: Angular =%f\n",self.fac1,self.fac2)


            if self.left == 1:
                self.fac2 += 0.05
                rospy.loginfo("Linear = %f: Angular =%f\n",self.fac1,self.fac2)

            elif self.right == -1:
                self.fac2 -= 0.05
                rospy.loginfo("Linear = %f: Angular =%f\n",self.fac1,self.fac2)


            twist.linear.x = self.fac1*self.aL
            twist.angular.z = self.fac2*self.aR

            if self.L1 == 1:    
                twist.angular.z = self.fac2*self.L1
            elif self.R1 == 1:    
                twist.angular.z = -self.fac2*self.R1 

            self.pub.publish(twist)
            rate.sleep()


if __name__ == "__main__":
    TJ = TwistJoy()
    rospy.spin()