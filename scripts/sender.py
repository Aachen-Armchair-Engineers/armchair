#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def sender():
    pub = rospy.Publisher('mockdata', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        data = "door handle at (x, y), %s" % rospy.get_time()
        rospy.loginfo("data sent")
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
