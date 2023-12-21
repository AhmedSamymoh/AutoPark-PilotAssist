#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int32
import serial
import signal
import sys

ser = None

# Define a signal handler function to gracefully handle Ctrl+C
def signal_handler(sig, frame):
    global ser
    rospy.loginfo("Received Ctrl+C, shutting down...")
    if ser is not None and ser.is_open:
        ser.close()
    sys.exit(0)

# Define a function to read data from the serial port
def read_serial(port='/dev/ttyUSB0', baudrate=9600):
    global ser
    try:
        # Open the serial port
        ser = serial.Serial(port, baudrate, timeout=1)
        
        # Initialize a ROS node
        rospy.init_node('serial_node', anonymous=True)
        
        # Create publishers for different topics
        pub_distance_forward = rospy.Publisher('distance_forward', Int32, queue_size=10)
        pub_distance_behind = rospy.Publisher('distance_behind', Int32, queue_size=10)
        pub_distance_back = rospy.Publisher('distance_back', Int32, queue_size=10)
        pub_angle_z = rospy.Publisher('angle_z', Float32, queue_size=10)
        pub_distance_D = rospy.Publisher('distance_D', Float32, queue_size=10)

        # Print information about the serial port
        print(f"Reading data from {port} at {baudrate} baud...")

        # Register the signal handler for Ctrl+C
        signal.signal(signal.SIGINT, signal_handler)

        # Main loop to continuously read and process data from the serial port
        while not rospy.is_shutdown():
            # Read a line from the serial port and decode it
            line = ser.readline().decode('utf-8').strip()

            # Check if a line is received
            if line:
                rospy.loginfo(f"Received: {line}")

                # Check if the line includes the distance information (D)
                if line.startswith('D:'):
                    distance_D = float(line.split(' ')[1])
                    pub_distance_D.publish(distance_D)
                elif line.startswith('distForw:'):
                    distForw = float(line.split(' ')[1])
                    pub_distance_forward.publish(int(distForw))
                elif line.startswith('distBehind:'):
                    distBehind = float(line.split(' ')[1])
                    pub_distance_behind.publish(int(distBehind))
                elif line.startswith('distBack:'):
                    distBack = float(line.split(' ')[1])
                    pub_distance_back.publish(int(distBack))
                elif line.startswith('ANGLE Z:'):
                    ANGLE = float(line.split(' ')[2])
                    pub_angle_z.publish(ANGLE)

    except serial.SerialException as e:
        rospy.logerr(f"Error: {e}")

    finally:
        # Close the serial port when the script is terminated
        if ser is not None and ser.is_open:
            ser.close()
            rospy.loginfo("Serial port closed.")

# Entry point of the script
if __name__ == "__main__":
    try:
        # Call the read_serial function
        read_serial()
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
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

# Define callback function for /distance_forward topic'
def callback_distance_forward(data):
    rospy.loginfo("Distance Forward: %d", data.data)
def distance_callback(msg):
    distance = msg.data
    rospy.loginfo(f"Distance Forward: {distance} meters")

    if distance > 20:
        # Move forward
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        rospy.loginfo("Car is moving forward")
    elif 14 <= distance <= 20:
        # Stop
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        rospy.loginfo("Car is stopped")
    # Add more conditions for other movements based on your requirements

# Subscribe to /distance_forward topic
rospy.Subscriber('/distance_forward', Float64, distance_callback)

# Spin ROS node
rospy.spin()

# Clean up GPIO on script exit
try:
    GPIO.cleanup()
except GPIO.error as e:
    rospy.logerr(f"GPIO cleanup failed: {e}")
