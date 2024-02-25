#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO

# GPIO pins for L298D motor driver
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23

EN1 = 13
EN2 = 12

# PID constants
kp = .70  # Proportional gain
ki = 0.0  # Integral gain
kd = 0.0  # Derivative gain

# Global variables for PID control
default_dist = 20

measure = 0

error_sum = 0.0
prev_error = 0.0



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

# Define callback function for /distance_forward topic
def callback_distance_forward(data):
    global measure
    measure = data.data
    rospy.loginfo("Distance Forward: %d", data.data)

def PIDcontrol():
    try:
        rospy.init_node('Auto_Pilot', anonymous=True)
        rospy.loginfo("ROS node initialized successfully")
        rospy.Subscriber('/distance_forward', Int32, callback_distance_forward)

        setup_gpio()

        while not rospy.is_shutdown():
        
            global error_sum, prev_error, kd, ki, kp, derivative
            error = measure - default_dist
            error_sum += error
            derivative = error - prev_error
            output = kp * error + ki * error_sum + kd * derivative

            # Calculate duty cycle
            speed = 10 + abs(output)
            duty_cycle = min(100.0, max(0.0, speed))  # Ensure duty cycle is between 0.0 and 100.0

            

            if 100>measure>20:
                pi_pwm.ChangeDutyCycle(duty_cycle)
                pi_pwm2.ChangeDutyCycle(duty_cycle)
                GPIO.output(IN1, GPIO.HIGH) 
                GPIO.output(IN2, GPIO.LOW) 
                GPIO.output(IN3, GPIO.LOW) 
                GPIO.output(IN4, GPIO.HIGH)  
            elif 19 > measure > 2:
                pi_pwm.ChangeDutyCycle(20)
                pi_pwm2.ChangeDutyCycle(20)
                GPIO.output(IN1, GPIO.LOW) 
                GPIO.output(IN2, GPIO.HIGH) 
                GPIO.output(IN3, GPIO.HIGH) 
                GPIO.output(IN4, GPIO.LOW) 
            else:
                GPIO.output(IN1, GPIO.LOW) 
                GPIO.output(IN2, GPIO.LOW) 
                GPIO.output(IN3, GPIO.LOW) 
                GPIO.output(IN4, GPIO.LOW) 

                prev_error = error
            

    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException: Shutting down")
    except Exception as e:
        rospy.logerr(f"An error occurred: {str(e)}")
    finally:
        GPIO.output(IN1, GPIO.LOW) 
        GPIO.output(IN2, GPIO.LOW) 
        GPIO.output(IN3, GPIO.LOW) 
        GPIO.output(IN4, GPIO.LOW) 
        GPIO.cleanup()

if __name__ == '__main__': 
    PIDcontrol()

    
