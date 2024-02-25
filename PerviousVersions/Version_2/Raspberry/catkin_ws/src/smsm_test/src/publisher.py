#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def publisher():
    rospy.init_node('hello_publisher', anonymous=True)
    pub = rospy.Publisher('hello_topic', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        msg = "Hello, SMSM!"
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
