#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int32
import RPi.GPIO as GPIO

# GPIO pins for L298D motor driver
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
EN1 = 18
EN2 = 25

# Disable GPIO warnings
GPIO.setwarnings(False)

# Set up GPIO
try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)
except GPIO.error as e:
    rospy.logerr(f"GPIO setup failed: {e}")

# Set up ROS node
rospy.init_node('auto_pilot')

# Define callback function for /distance_forward topic
def callback_distance_forward(data):
    rospy.loginfo("Distance Forward: %d", data.data)

    if data.data > 20:
        # Move forward
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        rospy.loginfo("Car is moving forward")
    elif 14 <= data.data <= 20:
        # Stop
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        rospy.loginfo("Car is stopped")
    # Add more conditions for other movements based on your requirements

# Subscribe to /distance_forward topic
rospy.Subscriber('/distance_forward', Int32, callback_distance_forward)

# Spin ROS node
rospy.spin()

# Clean up GPIO on script exit
try:
    GPIO.cleanup()
except GPIO.error as e:
    rospy.logerr(f"GPIO cleanup failed: {e}")
