#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import sys
import tty
import termios
from time import sleep
# GPIO pins for L298D motor driver
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
D1 = 0
D2 = 0
EN1 = 13
EN2 = 12
Difference1 = 0
Difference2 = 0

speed1 = 30
speed2 = 30

speed = 30

pi_pwm = None
pi_pwm2 = None

def setup_gpio():
    GPIO.setmode(GPIO.BCM)  # Set pin numbering mode to Broadcom (BCM) mode
    GPIO.setwarnings(False)  # Disable warnings
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)
    GPIO.setup(EN1, GPIO.OUT)
    GPIO.setup(EN2, GPIO.OUT)

    global pi_pwm, pi_pwm2
    pi_pwm = GPIO.PWM(EN1, 1000)
    pi_pwm2 = GPIO.PWM(EN2, 1000)
    pi_pwm.start(0)
    pi_pwm2.start(0)




def stop_motor():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)


def control_motor(direction):
    global speed1, speed2
    if direction == 'd':  # move right
        speed1 = 35
        speed2 = 35
        pi_pwm.ChangeDutyCycle(speed1)
        pi_pwm2.ChangeDutyCycle(speed2)
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)

    elif direction == 'a':  # move left
        speed1 = 35
        speed2 = 35
        pi_pwm.ChangeDutyCycle(speed1)
        pi_pwm2.ChangeDutyCycle(speed2)
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)

    elif direction == 's':  # back
        pi_pwm.ChangeDutyCycle(speed)
        pi_pwm2.ChangeDutyCycle(speed)
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)

    elif direction == 'w':  # forward
        pi_pwm.ChangeDutyCycle(30)
        pi_pwm2.ChangeDutyCycle(28)
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)

    elif direction == 'b':  # b for break
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
