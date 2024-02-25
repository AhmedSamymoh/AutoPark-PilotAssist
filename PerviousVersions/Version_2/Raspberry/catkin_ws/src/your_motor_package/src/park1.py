#!/usr/bin/env python3 
 
import rospy 
from std_msgs.msg import String 
import RPi.GPIO as GPIO 
import sys 
import tty 
import termios 
import RPi.GPIO as GPIO 
from time import sleep 
from std_msgs.msg import String, Int32 
 
# GPIO pins for L298D motor driver 
IN1 = 17 
IN2 = 27 
IN3 = 22 
IN4 = 23 

EN1 = 13 
EN2 = 12 

Difference1 = 0 
Difference2 = 0 

D1 = 0 
D2 = 0 
 
speed1 = 20
speed2 = 20

speed = 20

pi_pwm = None
pi_pwm2 = None


def callback_distance_behind1(data):
    global D1
    D1 = data.data
    rospy.loginfo("Distance Behind1: %d", data.data)


def callback_distance_behind2(data):
    global D2
    D2 = data.data
    rospy.loginfo("Distance Behind2: %d", data.data)    


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
 
 
def turn_right(): 
    speed1 = 10
    speed2 = 30
    pi_pwm.ChangeDutyCycle(speed1)
    pi_pwm2.ChangeDutyCycle(speed2)
    
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    
def rotate_right(): 
    speed1 = 20
    speed2 = 20
    pi_pwm.ChangeDutyCycle(speed1)
    pi_pwm2.ChangeDutyCycle(speed2)
    
    GPIO.output(IN1, GPIO.HIGH) 
    GPIO.output(IN2, GPIO.LOW) 
    GPIO.output(IN3, GPIO.HIGH) 
    GPIO.output(IN4, GPIO.LOW)  
      
def turn_left(): 
    speed1 = 30
    speed2 = 10
    pi_pwm.ChangeDutyCycle(speed1)
    pi_pwm2.ChangeDutyCycle(speed2)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    
def rotate_left(): 
    speed1 = 30
    speed2 = 30
    pi_pwm.ChangeDutyCycle(speed1)
    pi_pwm2.ChangeDutyCycle(speed2)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    
    
def move_backward(): 
    pi_pwm.ChangeDutyCycle(speed)
    pi_pwm2.ChangeDutyCycle(speed)
    GPIO.output(IN1, GPIO.LOW) 
    GPIO.output(IN2, GPIO.HIGH) 
    GPIO.output(IN3, GPIO.HIGH) 
    GPIO.output(IN4, GPIO.LOW) 
    
def move_forward(): 
    pi_pwm.ChangeDutyCycle(speed)
    pi_pwm2.ChangeDutyCycle(speed)
    GPIO.output(IN1, GPIO.HIGH) 
    GPIO.output(IN2, GPIO.LOW) 
    GPIO.output(IN3, GPIO.LOW) 
    GPIO.output(IN4, GPIO.HIGH) 
 

def move_aline_wall():
    global Difference1, Difference2, D1, D2  # Fix 5: Declare global variables
    Difference1 = D1 - D2
    Difference2 = D2 - D1
    while Difference1 >= 3:
        turn_left()
        Difference1 = D1 - D2  # Fix: Update the value of Difference1
        if D1 > 30 and D2 > 30:
            stop_motor()
            break

    while Difference2 >= 3:
        turn_right()
        Difference2 = D2 - D1  # Fix: Update the value of Difference2
        if D1 > 30 and D2 > 30:
            stop_motor()
            break

    while 0 < Difference1<1 or 0 < Difference2 < 1 or D1 == D2:
        move_forward()
        Difference1 = D1 - D2  # Fix: Update the value of Difference1
        Difference2 = D2 - D1  # Fix: Update the value of Difference2
        if D1 >30 and D2 > 30:
            stop_motor()
            break

    
def control():
    try:
        rospy.init_node('Auto_Parking', anonymous=True)
        rospy.loginfo("ROS node initialized successfully")

        rospy.Subscriber('/distance_behind1', Int32, callback_distance_behind1)
        rospy.Subscriber('/distance_behind2', Int32, callback_distance_behind2)

        setup_gpio()

        while not rospy.is_shutdown():
            if D1 < 30 and D2 < 30:
                move_aline_wall()
            else:
                stop_motor()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException: Shutting down")
    except Exception as e:
        rospy.logerr(f"An error occurred: {str(e)}")
    finally:
        stop_motor()
        GPIO.cleanup()

if __name__ == '__main__': 
    control() 
 
