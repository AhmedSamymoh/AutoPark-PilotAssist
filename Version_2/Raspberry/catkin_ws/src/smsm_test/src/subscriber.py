#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(f"I heard: {data.data}")

def subscriber():
    rospy.init_node('hello_subscriber', anonymous=True)
    rospy.Subscriber('hello_topic', String, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
