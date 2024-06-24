#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Int32
import struct

class RVizVisualizationNode:
    def __init__(self):
        rospy.init_node('rviz_visualization_node', anonymous=True)
        self.pub_point_cloud = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
        rospy.Subscriber('distance_forward', Int32, self.distance_forward_callback)
        rospy.Subscriber('distance_behind1', Int32, self.distance_behind1_callback)
        rospy.Subscriber('distance_behind2', Int32, self.distance_behind2_callback)
        rospy.Subscriber('distance_back', Int32, self.distance_back_callback)

        self.cloud = []

    def distance_to_point(self, distance, direction):
        if direction == "forward":
            return 0, distance, 0
        elif direction == "behind1":
            return 0, -distance, 0
        elif direction == "behind2":
            return 0, -distance, 0
        elif direction == "back":
            return 0, distance, 0
        else:
            return 0, 0, 0

    def append_to_cloud(self, x, y, z):
        self.cloud.append([x, y, z])

    def publish_point_cloud(self):
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = rospy.Time.now()
        cloud_msg.header.frame_id = 'base_link'  # Adjust frame ID as necessary

        cloud_msg.height = 1
        cloud_msg.width = len(self.cloud)
        cloud_msg.fields.append(PointField('x', 0, PointField.FLOAT32, 1))
        cloud_msg.fields.append(PointField('y', 4, PointField.FLOAT32, 1))
        cloud_msg.fields.append(PointField('z', 8, PointField.FLOAT32, 1))
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        cloud_msg.is_bigendian = False

        cloud_msg.data = []
        for point in self.cloud:
            cloud_msg.data.extend(struct.pack('<fff', *point))

        self.pub_point_cloud.publish(cloud_msg)
        self.cloud = []  # Clear the cloud after publishing

    def distance_forward_callback(self, data):
        distance = data.data
        x, y, z = self.distance_to_point(distance, "forward")
        self.append_to_cloud(x, y, z)
        self.publish_point_cloud()

    def distance_behind1_callback(self, data):
        distance = data.data
        x, y, z = self.distance_to_point(distance, "behind1")
        self.append_to_cloud(x, y, z)
        self.publish_point_cloud()

    def distance_behind2_callback(self, data):
        distance = data.data
        x, y, z = self.distance_to_point(distance, "behind2")
        self.append_to_cloud(x, y, z)
        self.publish_point_cloud()

    def distance_back_callback(self, data):
        distance = data.data
        x, y, z = self.distance_to_point(distance, "back")
        self.append_to_cloud(x, y, z)
        self.publish_point_cloud()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rviz_node = RVizVisualizationNode()
    rviz_node.run()
