#!/usr/bin/env python3
import rospy
from joy_pwm import PWMJoy
from joy_twist_tele import TwistJoy
from sensor_msgs.msg import Joy
from termcolor import colored
from ascii import welcome


colors = ['red', 'green', 'cyan', 'blue', 'magenta']

for i in range(len(welcome)):
    print(colored(welcome[i], colors[i%5]),end="")
print()

print(colored("Press Y for Twist To PWM Mode", color=colors[2]))
print(colored("Press X for Tank Mode", color=colors[1]))

keyY =0
keyX =0

def callback(data):
    global keyX, keyY
    bt = data.buttons
    keyX = bt[2]
    keyY = bt[3]

def start():
    global keyX, keyY
    rate = rospy.Rate(2)

    
    rospy.Subscriber("/joy", Joy, callback)
    while not rospy.is_shutdown():
        print(keyX)

        if keyY == 1:
            TJ = TwistJoy()
        if keyX == 1:
            PJ = PWMJoy()
        rate.sleep()
        
start()


