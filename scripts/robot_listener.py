#!/usr/bin/env python3

import sys
import copy
import rospy
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

arm_group =  moveit_commander.MoveGroupCommander("arm")
gripper_group =  moveit_commander.MoveGroupCommander("gripper")

arm_group.set_max_velocity_scaling_factor(0.5)

tfBuffer = tf2_ros.Buffer()
#tfListener = tf2_ros.TransformListener()

def move_robot_to_position(p):
    rospy.loginfo("Moving arm")
    pose = arm_group.get_current_pose().pose
    pose.position = p
    arm_group.set_pose_target(pose)
    arm_group.plan()
    #arm_group.go(wait=True)
    rospy.loginfo("finished")

def callback(data):
    ps = data
    
    rospy.loginfo(ps.point)

    tfBuffer.transform(ps, "/base")
    move_robot_to_position(ps.point)
#    Point transformed = 

    rospy.loginfo(ps.point)
    
def listener():
    rospy.init_node('receiver', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber('armchair/door_position', PointStamped, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
