#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy

class PWMJoy:
    def __init__(self):        
        rospy.init_node('Joy2PWM')
        rospy.Subscriber("/joy", Joy, self.callback)    
        self.left_pwm_pub = rospy.Publisher('left_pwm', Int32, queue_size=10)
        self.right_pwm_pub = rospy.Publisher('right_pwm', Int32, queue_size=10)
        self.max_pwm_val = 180
        self.min_pwm_val = -180
        self.fac1 = 170
        self.fac2 = 80
        self.fb1 = 0
        self.lr1 = 0
        self.fb2 = 0
        self.lr2 = 0
        self.up = 0
        self.down = 0
        self.left = 0
        self.right = 0
        self.start()

    def get_pwm(self, L1, L2, R1, R2):
    
        PWML1 = max(min(L1, self.max_pwm_val), self.min_pwm_val)
        PWML2 = max(min(L2,self.max_pwm_val), self.min_pwm_val)
        PWMR1 = max(min(R1, self.max_pwm_val), self.min_pwm_val)
        PWMR2 = max(min(R2,self.max_pwm_val), self.min_pwm_val)

        return PWML1, PWML2, PWMR1, PWMR2
    
    def callback(self,data):    
        self.axes = data.axes
        self.buttons = data.buttons
        self.fb1 = data.axes[1]
        self.lr1 = data.axes[0]
        self.fb2 = data.axes[4]
        self.lr2 = data.axes[3]
        self.up = self.axes[7]
        self.down = self.axes[7]
        self.left = self.axes[6]
        self.right = self.axes[6]
        
        


    def start(self):
        rate = rospy.Rate(15)
        self.left_pwm = Int32()
        self.right_pwm = Int32()
        while not rospy.is_shutdown():
            if self.up == 1:
                self.fac1 += 1
                rospy.loginfo("max fac1 = %f: max fac 2=%f\n",self.fac1,self.fac2)

            elif self.down == -1:
                self.fac1 -= 1
                rospy.loginfo("max fac1 = %f: max fac 2=%f\n",self.fac1,self.fac2)

            if self.left == 1:
                self.fac2 += 1
                rospy.loginfo("max fac1 = %f: max fac 2=%f\n",self.fac1,self.fac2)

            elif self.right == -1:
                self.fac2 -= 1
                rospy.loginfo("max fac1 = %f: max fac 2=%f\n",self.fac1,self.fac2)


            pwmL1 = self.fac1*self.fb1
            pwmR1 = self.fac1*self.fb2
            pwmL2 = self.fac2*self.lr1
            pwmR2 = self.fac2*self.lr2
            left_data1, left_data2, right_data1, right_data2 =self.get_pwm(pwmL1, pwmL2, pwmR1, pwmR2)
            if left_data1 >=0 and left_data2 >=0:
                left_data = int(max(left_data1, left_data2))
            else:
                left_data = int(min(left_data1, left_data2))
            if right_data1 >=0 and right_data2 >=0:
                right_data = int(max(right_data1, right_data2))
            else:
                right_data = int(min(right_data1,right_data2))

                

            self.left_pwm_pub.publish(left_data)
            self.right_pwm_pub.publish(right_data)            
            
            rate.sleep()

if __name__ == "__main__":
    PJ = PWMJoy()
    rospy.spin()
