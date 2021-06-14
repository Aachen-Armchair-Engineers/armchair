#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

arm_group =  moveit_commander.MoveGroupCommander("arm")
gripper_group =  moveit_commander.MoveGroupCommander("gripper")

#display_trajectory_publisher = rospy.Publisher(
#    "/move_group/display_planned_path",
#    moveit_msgs.msg.DisplayTrajectory,
#    queue_size=20,
#)

# We can get the name of the reference frame for this robot:
planning_frame = arm_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = arm_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")


arm_group.set_max_velocity_scaling_factor(1.0)

print("Arm_pose:")
print(arm_group.get_current_pose())
print("Gripper joints:")
print(gripper_group.get_current_joint_values())
print("Moving arm")
pose = arm_group.get_current_pose().pose
pose.position.x += 0.1
arm_group.set_pose_target(pose)
arm_group.plan()
#arm_group.go(wait=True)
#pose.position.y += 0.1
#arm_group.set_pose_target(pose)
#arm_group.go(wait=True)
#pose.position.z += 0.1
#arm_group.set_pose_target(pose)
#arm_group.go(wait=True)
print("finished")

#print("Moving Gripper")
#joint_goal = [0.5, 0.5, 0.5]
#gripper_group.go(joint_goal, wait=True)
#joint_goal = [0.0, 0.0, 0.0]
#gripper_group.go(joint_goal, wait=True)
#gripper_group.set_joint_value_target("j2n6s300_joint_finger_1", 0.5)
#gripper_group.set_joint_value_target("j2n6s300_joint_finger_2", 0.0)
#gripper_group.go(wait=True)
#print("Finished")

