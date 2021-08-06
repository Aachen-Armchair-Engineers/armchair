#!/usr/bin/env python3

import sys
#from math import pi, tau, dist, fabs, cos
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PointStamped, Quaternion
import tf2_ros
#from tf2_geometry_msgs import PointStamped
import moveit_commander
import moveit_msgs.msg
from tf.transformations import quaternion_from_euler

robot = None
arm_group = None
gripper_group = None

tfBuffer = None
tfListener = None

last_pos = None

target_frame = None 

#TODO: Use this for the entire opening process
#See: http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html#displaying-a-trajectory
# https://answers.ros.org/question/278616/how-to-create-a-publisher-about-trajectory-path-then-show-it-in-rviz/
# http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
display_trajectory_publisher = rospy.Publisher(
       "/move_group/display_planned_path",
       moveit_msgs.msg.DisplayTrajectory,
       queue_size=20,
    )

def init_robot():
    global arm_group, gripper_group, last_pos, target_frame, robot

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    arm_group =  moveit_commander.MoveGroupCommander("arm")
    gripper_group =  moveit_commander.MoveGroupCommander("gripper")

    arm_group.set_max_velocity_scaling_factor(0.3)

    target_frame = arm_group.get_planning_frame()
    rospy.logdebug("Robot planning frame is %s" % target_frame)


def get_robot_info():
    # We can get the name of the reference frame for this robot:
    planning_frame = arm_group.get_planning_frame()
    rospy.logdebug("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = arm_group.get_end_effector_link()
    rospy.logdebug("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    rospy.logdebug("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    rospy.logdebug("============ Printing robot state")
    rospy.logdebug(robot.get_current_state())
    rospy.logdebug("")

    rospy.logdebug("Arm_pose:")
    rospy.logdebug(arm_group.get_current_pose())
    rospy.logdebug("Gripper joints:")
    rospy.logdebug(gripper_group.get_current_joint_values())

def move_robot_test():
    rospy.logdebug("Moving arm")
    pose = arm_group.get_current_pose().pose

    pose.position.x += 0.1
    arm_group.set_pose_target(pose)
    arm_group.plan() #Why is this not needed for the other move commands?
    arm_group.go(wait=True)

    pose.position.y += 0.1
    arm_group.set_pose_target(pose)
    arm_group.go(wait=True)

    pose.position.z += 0.1
    arm_group.set_pose_target(pose)
    arm_group.go(wait=True)
    rospy.logdebug("finished")

    rospy.logdebug("Moving Gripper")
    robot_open_hand()
    robot_close_hand()
    rospy.logdebug("Finished")

def robot_open_hand():
    joint_goal = [0.5, 0.5, 0.5]
    gripper_group.go(joint_goal, wait=True)

def robot_close_hand():
    joint_goal = [0.0, 0.0, 0.0]
    gripper_group.go(joint_goal, wait=True)

def robot_plan_with_offset(offset = 0.0):
    global tfBuffer, tfListener
    global target_frame, last_pos

    if last_pos is None:
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
    rospy.logwarn("NOT IMPLEMENTED YET")

def robot_plan_circular_motion():
    rospy.logwarn("NOT IMPLEMENTED YET")

    radius_door = 0.8
    radius_handle = 0.7
    directions = ["left", "right"]

    #TODO: How to cartesian a circle
    # https://answers.ros.org/question/211343/how-to-use-moveit-for-planning-circular-path-with-a-robotic-arm/
    # http://docs.ros.org/en/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#ad6b02d15000d5b17c89b15a0f744b47c


def robot_go():
    arm_group.go(wait=True)
    rospy.loginfo("finished")

def robot_routine():
    #Move fast to the position before the handle
    arm_group.set_max_velocity_scaling_factor(0.3)

    robot_plan_with_offset(0.2)

    robot_go()


    #Open the gripper
    robot_open_hand()


    #Slowly move the last distance
    arm_group.set_max_velocity_scaling_factor(0.1)

    robot_plan_with_offset(0.1)

    robot_go()


    #close the gripper 
    robot_close_hand()

    
    # Move back to the previous position
    robot_plan_with_offset(0.2)
    #TODO: use this instead:
    #robot_plan_circular_motion()

    robot_go()


    #Open the gripper again
    robot_open_hand()


    #Go back into neutral position
    arm_group.set_max_velocity_scaling_factor(0.3)

    robot_plan_homing()

    robot_go()


#called once we send a signal manually to plan&move 
def btn_callback(data):
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

    elif data.data == 20:
        rospy.loginfo("full routine")
        robot_routine()

#called every time we detect an object
def pos_callback(data):
    global last_pos
    last_pos = data
    
    #last_pos = tfBuffer.transform(ps, "root").point


def listener():
    global tfBuffer, tfListener

    rospy.init_node('robot_interface', anonymous=True)
    #rate = rospy.Rate(10)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    init_robot()

    rospy.loginfo("Starting robot listening")

    rospy.Subscriber('armchair/handle_position', PointStamped, pos_callback)
    rospy.Subscriber('armchair/cmd', Int32, btn_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
