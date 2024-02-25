#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int32
import serial
import signal
import sys
import select  # Import the select module for checking input

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
        pub_distance_behind1 = rospy.Publisher('distance_behind1', Int32, queue_size=10)
        pub_distance_behind2 = rospy.Publisher('distance_behind2', Int32, queue_size=10)
        pub_distance_back = rospy.Publisher('distance_back', Int32, queue_size=10)
        pub_angle_z = rospy.Publisher('angle_z', Float32, queue_size=10)
        pub_distance_D = rospy.Publisher('distance_D', Float32, queue_size=10)

        # Print information about the serial port
        print(f"Reading data from {port} at {baudrate} baud...")

        # Register the signal handler for Ctrl+C
        signal.signal(signal.SIGINT, signal_handler)

        # Main loop to continuously read and process data from the serial port
        while not rospy.is_shutdown():
            # Check if there's input available on the terminal
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                # Read a line from the terminal
                input_line = sys.stdin.readline().strip()

                # Check if the input line is in the correct format (e.g., "150 150")
                if len(input_line.split()) == 2:
                    # Parse the input values
                    speed1, speed2 = map(int, input_line.split())

                    # Send the speeds to the ESP32 via serial
                    ser.write(f"{speed1} {speed2}\n".encode())

                    # Log the sent speeds
                    rospy.loginfo(f"Sent speeds to ESP32: Speed1={speed1}, Speed2={speed2}")

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
                elif line.startswith('distBehind1:'):
                    distBehind1 = float(line.split(' ')[1])
                    pub_distance_behind1.publish(int(distBehind1))
                elif line.startswith('distBehind2:'):
                    distBehind2 = float(line.split(' ')[1])
                    pub_distance_behind2.publish(int(distBehind2))
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
