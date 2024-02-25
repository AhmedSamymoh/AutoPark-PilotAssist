#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped
import serial

def parse_serial_data(data):
    try:
        # Split the data string into individual elements
        parts = data.split(',')
        
        # Check if the received data has the expected format
        if len(parts) == 2:
            # Extract values for AccZ and GyroZ
            acc_z = float(parts[0].split(':')[-1].strip())
            gyro_z = float(parts[1].split(':')[-1].strip())

            # Log the received data
            rospy.loginfo(f"Received data: AccZ: {acc_z}, GyroZ: {gyro_z}")

            return acc_z, gyro_z
        else:
            # Log a warning for unexpected data format
            rospy.logwarn(f"Received unexpected data format: {data}")
            return None, None
    except Exception as e:
        # Log an error if there is an issue parsing the data
        rospy.logerr(f"Error parsing serial data: {e}")
        return None, None

def axis_node():
    rospy.init_node('axis_node', anonymous=True)
    pub = rospy.Publisher('axis_data', Vector3Stamped, queue_size=10)

    with serial.Serial('/dev/ttyUSB0', 9600) as ser:  # Change '/dev/ttyUSB0' to your Arduino port
        while not rospy.is_shutdown():
            try:
                raw_data = ser.readline().decode('utf-8').strip()
                acc_z, gyro_z = parse_serial_data(raw_data)
                if acc_z is not None and gyro_z is not None:
                    # Create a Vector3Stamped message
                    axis_msg = Vector3Stamped()
                    axis_msg.header.stamp = rospy.Time.now()
                    axis_msg.header.frame_id = 'base_link'  # Change this frame ID if needed
                    axis_msg.vector.x = acc_z  # Use AccZ as the rotation angle
                    axis_msg.vector.y = gyro_z  # Use GyroZ as the angular velocity
                    axis_msg.vector.z = 0.0  # Set Z-axis to 0 for simplicity

                    # Publish the message
                    pub.publish(axis_msg)

            except Exception as e:
                rospy.logerr(f"Error reading serial data: {e}")

if __name__ == '__main__':
    try:
        axis_node()
    except rospy.ROSInterruptException:
        pass
