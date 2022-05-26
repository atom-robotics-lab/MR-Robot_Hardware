#! /usr/bin/env python3


import RPi.GPIO as GPIO
from time import sleep
import rospy
from std_msgs.msg import Int16

seq_a_lw = 0
seq_b_lw = 0
count_rgt_lw = 0  # clk wise
count_lft_lw = 0  # anit_clk wise

seq_a_rw = 0
seq_b_rw = 0
count_rgt_rw = 0  # clk wise
count_lft_rw = 0  # anit_clk wise

cnt_lw = 0
cnt_rw = 0

lft_Enc_A = 17
lft_Enc_B = 27

rgt_Enc_A = 23
rgt_Enc_B = 24

wheel_rad = 0.094
ticks_in_1_rotation = 545
d_in_1_roatation = 0.5906  # in meter

fwrd_distance = 0
bkwrd_distance = 0


def algo(val_a, val_b, seq_a, seq_b):
    seq_a = seq_a << 1
    seq_a = seq_a | val_a
    seq_a = seq_a & 15

    seq_b = seq_b << 1
    seq_b = seq_b | val_b
    seq_b = seq_b & 15

    return [seq_a, seq_b]


def init():
    print("Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(rgt_Enc_B, GPIO.IN)
    GPIO.setup(rgt_Enc_A, GPIO.IN)
    GPIO.setup(lft_Enc_A, GPIO.IN)
    GPIO.setup(lft_Enc_B, GPIO.IN)

    GPIO.add_event_detect(lft_Enc_B, GPIO.BOTH, callback=isr_B)
    GPIO.add_event_detect(lft_Enc_A, GPIO.BOTH, callback=isr_A)

    GPIO.add_event_detect(rgt_Enc_B, GPIO.BOTH, callback=isr_b)
    GPIO.add_event_detect(rgt_Enc_A, GPIO.BOTH, callback=isr_a)

    return


def isr_A(lft_Enc_A):
    global seq_a_lw
    global seq_b_lw
    global count_lft_lw
    global count_rgt_lw
    global cnt_lw
    global fwrd_distance
    global bkwrd_distance

    a_val = GPIO.input(lft_Enc_A)
    b_val = GPIO.input(lft_Enc_B)

    seq_a_lw = seq_a_lw << 1
    seq_a_lw = seq_a_lw | a_val
    seq_a_lw = seq_a_lw & 15

    seq_b_lw = seq_b_lw << 1
    seq_b_lw = seq_b_lw | b_val
    seq_b_lw = seq_b_lw & 15
    # anti_clk wise
    if (seq_a_lw == 9) & (seq_b_lw == 3):
        count_lft_lw = count_lft_lw + 1
    # clk wise
    if (seq_a_lw == 3) & (seq_b_lw == 9):
        count_rgt_lw = count_rgt_lw + 1
    cnt_lw = count_lft_lw - count_rgt_lw
    # forward motion +ve
    print(f"counter_lw:{cnt_lw}")
    left_pub.publish(cnt_lw)
    if cnt_lw > 0:
        fwrd_distance = cnt_lw * d_in_1_roatation
        print(f"foward_distance:{round(fwrd_distance, 4)}")
    else:
        bkwrd_distance = abs(cnt_lw * d_in_1_roatation)
        print(f"backword_distance={round(bkwrd_distance, 4)}")


def isr_B(lft_Enc_B):
    global seq_a_lw
    global seq_b_lw
    global count_lft_lw
    global count_rgt_lw
    global cnt_lw

    a_val = GPIO.input(lft_Enc_A)
    b_val = GPIO.input(lft_Enc_B)
     
    seq_a_lw = seq_a_lw << 1
    seq_a_lw = seq_a_lw | a_val
    seq_a_lw = seq_a_lw & 15

    seq_b_lw = seq_b_lw << 1
    seq_b_lw = seq_b_lw | b_val
    seq_b_lw = seq_b_lw & 15

    # anti_clk wise
    if (seq_a_lw == 9) & (seq_b_lw == 3):
        count_lft_lw = count_lft_lw + 1
    # clk wise
    if (seq_a_lw == 3) & (seq_b_lw == 9):
        count_rgt_lw = count_rgt_lw + 1
    cnt_lw = count_lft_lw - count_rgt_lw
    # forward motion +ve
    print(f"counter_lw:{cnt_lw}")
    left_pub.publish(cnt_lw)

    if cnt_lw > 0:
        fwrd_distance = cnt_lw * d_in_1_roatation
        print(f"foward_distance:{round(fwrd_distance, 4)}")
    else:
        bkwrd_distance = abs(cnt_lw * d_in_1_roatation)
        print(f"backword_distance={round(bkwrd_distance, 4)}")


def isr_b(rgt_Enc_B):
    global seq_a_rw
    global seq_b_rw
    global count_lft_rw
    global count_rgt_rw
    global cnt_rw
    global fwrd_distance
    global bkwrd_distance
    a_val = GPIO.input(rgt_Enc_A)
    b_val = GPIO.input(rgt_Enc_B)
    print(a_val, b_val)

    seq_a_rw = seq_a_rw << 1
    seq_a_rw = seq_a_rw | a_val
    seq_a_rw = seq_a_rw & 15

    seq_b_rw = seq_b_rw << 1
    seq_b_rw = seq_b_rw | b_val
    seq_b_rw = seq_b_rw & 15
    print(seq_a_rw, seq_b_rw)

    # anti_clk wise
    if (seq_a_rw == 9) & (seq_b_rw == 3):
        count_lft_rw = count_lft_rw + 1
    # clk wise
    if (seq_a_rw == 3) & (seq_b_rw == 9):
        count_rgt_rw = count_rgt_rw + 1
    cnt_rw = count_lft_rw - count_rgt_rw
    # forward motion +ve
    print(f"counter_rw:{cnt_rw}")
    right_pub.publish(cnt_rw)


def isr_a(rgt_Enc_A):
    global seq_a_rw
    global seq_b_rw
    global count_lft_rw
    global count_rgt_rw
    global cnt_rw

    a_val = GPIO.input(rgt_Enc_A)
    b_val = GPIO.input(rgt_Enc_B)
    print(a_val, b_val)

    seq_a_rw = seq_a_rw << 1
    seq_a_rw = seq_a_rw | a_val
    seq_a_rw = seq_a_rw & 15

    seq_b_rw = seq_b_rw << 1
    seq_b_rw = seq_b_rw | b_val
    seq_b_rw = seq_b_rw & 15
    print(seq_a_rw, seq_b_rw)

    # anti_clk wise
    if (seq_a_rw == 9) & (seq_b_rw == 3):
        count_lft_rw = count_lft_rw + 1
    # clk wise
    if (seq_a_rw == 3) & (seq_b_rw == 9):
        count_rgt_rw = count_rgt_rw + 1
    cnt_rw = count_lft_rw - count_rgt_rw
    # forward motion +ve
    print(f"counter_rw:{cnt_rw}")
    right_pub.publish(cnt_rw)


def main():
    try:
        init()
        while True:
            sleep(1)

    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__ == '__main__':
    rospy.init_node("EncoderOdom", anonymous=True)

    right_pub = rospy.Publisher('right_count', Int16, queue_size=10)
    left_pub = rospy.Publisher('left_count', Int16, queue_size=10)

    main()
