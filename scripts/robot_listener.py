#!/usr/bin/env python3

import sys
import copy
import rospy
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Point, PointStamped, Quaternion
import tf2_ros
from tf2_geometry_msgs import PointStamped
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

arm_group = None
gripper_group = None

tfBuffer = None
tfListener = None

last_pos = None

target_frame = None 

def init_robot():
    global arm_group, gripper_group, last_pos, target_frame

    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    arm_group =  moveit_commander.MoveGroupCommander("arm")
    gripper_group =  moveit_commander.MoveGroupCommander("gripper")

    arm_group.set_max_velocity_scaling_factor(0.3)

    target_frame = arm_group.get_planning_frame()

    rospy.loginfo("Robot planning frame is %s" % target_frame)

#    last_pos = arm_group.get_current_pose().pose.position

#pub = rospy.Publisher('armchair/door_position_tf', PointStamped, queue_size=10)

def robot_open_hand():
    joint_goal = [0.5, 0.5, 0.5]
    gripper_group.go(joint_goal, wait=True)

def robot_close_hand():
    joint_goal = [0.0, 0.0, 0.0]
    gripper_group.go(joint_goal, wait=True)

def robot_plan_with_offset(offset = 0.0):
    global tfBuffer, tfListener
    global target_frame, last_pos

    if last_pos == None:
        return

    last_pos.header.stamp = rospy.Time.now()

    target_pos = tfBuffer.transform(last_pos, "root")
    #pub.publish(target_pos)

    pose = arm_group.get_current_pose().pose
    pose.position = target_pos.point

    pose.position.y += offset
    
    #Grip from the front
    q = quaternion_from_euler(90,90,0)
    pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
    
    rospy.loginfo(pose)

    arm_group.set_pose_target(pose)
    arm_group.plan()

def robot_plan_homing():
    print ("")

def robot_go():
    arm_group.go(wait=True)
    rospy.loginfo("finished")

#called every time we detect an object
def pos_callback(data):
    global last_pos
    last_pos = data
    
    #last_pos = tfBuffer.transform(ps, "root").point

#called once we send a signal manually to plan&move 
def btn_callback(data):
    rospy.loginfo("WWWWW")
    rospy.loginfo(data)
    if data.data == 0:
        rospy.loginfo("received plan signal for offset pos")
        robot_plan_with_offset(0.2)
    
    elif data.data == 1:
        rospy.loginfo("received plan signal for final pos")
        robot_plan_with_offset(0.1)

    elif data.data == 2:
        rospy.loginfo("received plan signal")
        robot_plan_homing()

    elif data.data == 3:
        rospy.loginfo("received go signal")
        robot_go()
    
    elif data.data == 10:
        rospy.loginfo("opening")
        robot_open_hand()
    
    elif data.data == 11:
        rospy.loginfo("closing")
        robot_close_hand()


def listener():
    global tfBuffer, tfListener

    rospy.init_node('robot_listener', anonymous=True)
    rate = rospy.Rate(10)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    init_robot()

    rospy.loginfo("Starting robot listening")

    rospy.Subscriber('armchair/door_position', PointStamped, pos_callback)
    rospy.Subscriber('armchair/button', Int32, btn_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
