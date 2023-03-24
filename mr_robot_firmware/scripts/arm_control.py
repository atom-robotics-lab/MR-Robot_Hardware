import Adafruit_PCA9685
import time
import rospy

class servo_Class:
    #"Channel" is the channel for the servo motor on PCA9685
    #"ZeroOffset" is a parameter for adjusting the reference position of the servo motor
    def init(self, Channel, ZeroOffset):
        self.buttons = []
        self.Channel = Channel
        self.ZeroOffset = ZeroOffset

        #Initialize Adafruit_PCA9685
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40)
        self.pwm.set_pwm_freq(int(60))

    # Angle setting
    def SetPos(self,pos):
        #PCA9685 controls angles with pulses, 150~650 of pulses correspond to 0~180° of angle
        pulse = int((650-150)/180*pos+150+self.ZeroOffset)
        self.pwm.set_pwm(self.Channel, 0, pulse)

    # End processing
    def Cleanup(self):
        #The servo motor is set at 90°.
        self.SetPos(int(90))
        print('90')

    def callback(self,data):
        global left_btn, right_btn, pick_btn, travel_btn, drop_btn
        self.buttons = data.buttons
        left_btn = self.buttons[4]
        right_btn = self.buttons[5]
        pick_btn = self.buttons[3]
        travel_btn = self.buttons[2]
        drop_btn = self.buttons[1] 

if __name__ == '__main__':
    servo_class = servo_Class()
    rospy.init_node('arm_control')
    rospy.Subscriber("/joy", Joy, servo_class.callback)

    
    Servo0 = servo_Class(Channel=0, ZeroOffset=0)
    Servo1 = servo_Class(Channel=1, ZeroOffset=0)
    Servo2 = servo_Class(Channel=2, ZeroOffset=0)



    try:
        while True:

            if left_btn == 1:
                servo_class.SetPos(90)
            if right_btn == 1:
                servo_class.SetPos(110)
            if travel_btn == 1:
                servo_class.SetPos(10)

    except KeyboardInterrupt:
        print("\nCtl+C")
    except Exception as e:
        print(str(e))
    finally:
        Servo0.Cleanup()

        print("\nexit program")
