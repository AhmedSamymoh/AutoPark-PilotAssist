#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from std_msgs.msg import Header
from geometry_msgs.msg import Point32

class RvizNode:
    def __init__(self):
        rospy.init_node('rviz_node', anonymous=True)

        # Create a publisher for PointCloud2
        self.pub_point_cloud = rospy.Publisher('ultrasonic_point_cloud', PointCloud, queue_size=10)

        # Create subscribers for the ultrasonic sensor data
        rospy.Subscriber('distance_forward', Int32, self.distance_forward_callback)
        rospy.Subscriber('distance_behind1', Int32, self.distance_behind1_callback)
        rospy.Subscriber('distance_behind2', Int32, self.distance_behind2_callback)
        rospy.Subscriber('distance_back', Int32, self.distance_back_callback)
        rospy.Subscriber('angle_z', Float32, self.angle_z_callback)

    def distance_forward_callback(self, data):
        # Implement the logic to convert distance_forward data to points
        # Replace these lines with the actual logic based on your sensor data
        x = 1.0
        y = 2.0
        z = 0.0
        self.publish_point_cloud(x, y, z)

    def distance_behind1_callback(self, data):
        # Implement the logic to convert distance_behind1 data to points
        # Replace these lines with the actual logic based on your sensor data
        x = 3.0
        y = 4.0
        z = 0.0
        self.publish_point_cloud(x, y, z)

    def distance_behind2_callback(self, data):
        # Implement the logic to convert distance_behind2 data to points
        # Replace these lines with the actual logic based on your sensor data
        x = 5.0
        y = 6.0
        z = 0.0
        self.publish_point_cloud(x, y, z)

    def distance_back_callback(self, data):
        # Implement the logic to convert distance_back data to points
        # Replace these lines with the actual logic based on your sensor data
        x = 7.0
        y = 8.0
        z = 0.0
        self.publish_point_cloud(x, y, z)

    def angle_z_callback(self, data):
        # Implement the logic to convert angle_z data to points
        # Replace these lines with the actual logic based on your sensor data
        x = 9.0
        y = 10.0
        z = 0.0
        self.publish_point_cloud(x, y, z)

    def publish_point_cloud(self, x, y, z):
        # Publish the PointCloud2 message
        point_cloud = PointCloud()
        point_cloud.header = Header()
        point_cloud.header.stamp = rospy.Time.now()
        point_cloud.header.frame_id = 'ultrasonic_sensor_frame'

        # Populate the PointCloud message with ultrasonic sensor data
        point = Point32(x, y, z)
        point_cloud.points.append(point)

        # You may also add intensity information if needed
        # intensity_values = [1.0]
        # intensity_channel = ChannelFloat32()
        # intensity_channel.name = "intensity"
        # intensity_channel.values = intensity_values
        # point_cloud.channels.append(intensity_channel)

        # Publish the PointCloud message
        self.pub_point_cloud.publish(point_cloud)

if __name__ == '__main__':
    try:
        rviz_node = RvizNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
