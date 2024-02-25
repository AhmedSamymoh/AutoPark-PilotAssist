#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import sys
import tty
import termios

# GPIO pins for L298D motor driver
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
EN1 = 18
EN2 = 25

# Motor speed
speed = 60

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)
    GPIO.setup(EN1, GPIO.OUT)
    GPIO.setup(EN2, GPIO.OUT)
    GPIO.output(EN1, GPIO.HIGH)
    GPIO.output(EN2, GPIO.HIGH)

def stop_motor():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def control_motor(direction):
    if direction == 'd': #move right
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    elif direction == 'a': #move left
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
    elif direction == 's': #back
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    elif direction == 'w': #forward
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
    elif direction == 'b': #b for break
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
    else:
        stop_motor()

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def keyboard_control():
    rospy.init_node('motor_controller', anonymous=True)
    setup_gpio()

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key == 'q':
                stop_motor()
                break
            else:
                control_motor(key)
    except Exception as e:
        print(e)
    finally:
        stop_motor()
        GPIO.cleanup()

if __name__ == '__main__':
    keyboard_control()
