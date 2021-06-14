#!/usr/bin/env python

import rospy, tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

def callback():
    broadcast = tf2_ros.TransformBroadcaster()
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = 'base'
    transfrom.child_frame_id = 'oak-d_frame'
    transform.transform.translatio = ( 1.0, 0.0, 0.0)
    tranform.transform.rotation = (0.0, 0.0, 0.0) 
    broadcast.sendTransform(broadcast)

def broadcaster():
    rospy.init_node("broadcaster")
    rospy.Subscriber('/tf',TransformStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        broadcaster()
    except rospy.ROSInterruptException:
        pass
