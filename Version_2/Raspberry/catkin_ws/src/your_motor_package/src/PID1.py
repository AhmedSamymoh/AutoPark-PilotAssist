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
kp = 1.0  # Proportional gain
ki = 0.1  # Integral gain
kd = 0.01  # Derivative gain

# Global variables for PID control
target_speed = 0.0
current_speed = 0.0
error_sum = 0.0
prev_error = 0.0

# Obstacle detection parameters
obstacle_threshold = 20  # Minimum distance to maintain from the obstacle (in cm)

# Disable GPIO warnings
GPIO.setwarnings(False)

# Set up GPIO
try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)
    GPIO.setup(EN1, GPIO.OUT)
    GPIO.setup(EN2, GPIO.OUT)

    pwm1 = GPIO.PWM(EN1, 50)  # Set PWM frequency to 50 Hz
    pwm2 = GPIO.PWM(EN2, 50)

    pwm1.start(0)  # Start PWM with duty cycle 0
    pwm2.start(0)
except GPIO.error as e:
    rospy.logerr(f"GPIO setup failed: {e}")

# Set up ROS node
rospy.init_node('auto_pilot')

# Define callback function for /distance_forward topic
def some_function_of_distance(distance):
    max_distance = 100  # Adjust this value based on your application
    max_speed = 100     # Adjust this value based on your application

    # Calculate target speed based on distance_forward
    target_speed = max(0, max_speed * (1 - distance / max_distance))

    return target_speed

# Define callback function for /distance_forward topic
def callback_distance_forward(data):
    global target_speed
    rospy.loginfo("Distance Forward: %d", data.data)

    # Calculate target speed based on obstacle avoidance
    target_speed = some_function_of_distance(data.data)

# Subscribe to /distance_forward topic
rospy.Subscriber('/distance_forward', Int32, callback_distance_forward)

# Implement your speed measurement logic here
def measure_speed():
    # Replace this with your actual speed measurement logic
    return 0.0

# Main control loop
rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    # Measure current speed
    current_speed = measure_speed()

    # Calculate error
    error = target_speed - current_speed

    # Update integral and derivative terms
    error_sum += error
    derivative = error - prev_error

    # PID control equation
    output = kp * error + ki * error_sum + kd * derivative

    # Adjust speed based on obstacle avoidance
    if obstacle_threshold < current_speed < obstacle_threshold + 20:
        # Slow down when close to the obstacle
        output *= 0.5

    # Apply PWM to control speed
    pwm_value = int(abs(output))
    if output >= 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)

    pwm1.ChangeDutyCycle(pwm_value)

    # Update previous error
    prev_error = error

    rate.sleep()

# Clean up GPIO on script exit
try:
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
except GPIO.error as e:
    rospy.logerr(f"GPIO cleanup failed: {e}")
