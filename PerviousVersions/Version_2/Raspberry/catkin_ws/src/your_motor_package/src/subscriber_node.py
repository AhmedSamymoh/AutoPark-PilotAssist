#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int32

def callback_distance_forward(data):
    rospy.loginfo("Distance Forward: %d", data.data)

def callback_distance_behind(data):
    rospy.loginfo("Distance Behind: %d", data.data)

def callback_distance_back(data):
    rospy.loginfo("Distance Back: %d", data.data)

def callback_angle_z(data):
    rospy.loginfo("Angle Z: %.2f", data.data)

def callback_distance_D(data):
    rospy.loginfo("Distance D: %.2f", data.data)

def subscriber_node():
    rospy.init_node('subscriber_node', anonymous=True)

    rospy.Subscriber('distance_forward', Int32, callback_distance_forward)
    rospy.Subscriber('distance_behind', Int32, callback_distance_behind)
    rospy.Subscriber('distance_back', Int32, callback_distance_back)
    rospy.Subscriber('angle_z', Float32, callback_angle_z)
    rospy.Subscriber('distance_D', Float32, callback_distance_D)

    rospy.spin()

if __name__ == "__main__":
    subscriber_node()
