#!/usr/bin/env python3
'''
This script gets selected data from the camera and moves the robot accordingly:
camera_interface.py -> robot_interface.py -> Kinova Jaco
'''

from math import pi, cos, sin
import sys
import copy
import rospy
import numpy
from std_msgs.msg import Int32, Header
from geometry_msgs.msg import Point, PointStamped, Quaternion, Pose, PoseStamped, PoseArray
import tf2_ros
from tf2_geometry_msgs import PointStamped, do_transform_pose
import moveit_commander
import moveit_msgs.msg
from tf.transformations import quaternion_matrix, quaternion_from_euler, translation_matrix, concatenate_matrices, quaternion_from_matrix, translation_from_matrix, rotation_matrix
from kinova_msgs.srv import *
from armchair.msg import Command

tfBuffer = None
tfListener = None

def pose_to_matrix(pose):
    '''Convert a ROS TF Pose message into a transformation matrix'''
    p = pose.position
    p = [p.x, p.y, p.z]
    translation = translation_matrix(p)
    r = pose.orientation
    r = [r.x, r.y, r.z, r.w]
    rotation = quaternion_matrix(r)
    return concatenate_matrices(translation, rotation)

def matrix_to_pose(matrix):
    '''Convert a transformation matrix into a ROS TF Pose message'''
    translation = translation_from_matrix(matrix)
    translation = Point(translation[0], translation[1], translation[2])
    rotation = quaternion_from_matrix(matrix)
    rotation = Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])

    return Pose(translation, rotation)

def flatten(t):
    '''Flattens a list of lists'''
    t = [item for sublist in t for item in sublist]
    return t

display_trajectory_publisher = rospy.Publisher( 
       "/move_group/display_planned_path",
       moveit_msgs.msg.DisplayTrajectory,
       queue_size=20,
    )

diplay_poses_publisher = rospy.Publisher(
        '/armchair/poses',
        PoseArray,
        queue_size=10
    )

class RobotCommander:
    '''Helper class for planning, visualizing and executing trajectories of the robot arm'''

    def visualize_plans(self, *plans):
        '''Publish the planned trajectories for visualization in rviz'''
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()

        for plan in plans:
            display_trajectory.trajectory.append(plan)

        display_trajectory_publisher.publish(display_trajectory)

    def visualize_waypoints(self, *waypoints):
        '''Publish the planned waypoints (for the robot trajectory) for visualization in rviz'''

        wps = flatten(waypoints)

        diplay_poses_publisher.publish( 
                PoseArray(
                    Header(
                        frame_id=self.planning_frame,
                        stamp=rospy.Time.now()
                    ),
                    wps
                )
            )

    def execute_plans(self, *plans):
        '''Execute the previously planned trajectories'''
        for plan in plans:
            self.arm_group.execute(plan)

        rospy.loginfo("finished")


    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.arm_group =  moveit_commander.MoveGroupCommander("arm")
        self.gripper_group =  moveit_commander.MoveGroupCommander("gripper")

        self.arm_group.set_max_velocity_scaling_factor(0.3)

        self.planning_frame = self.arm_group.get_planning_frame()

        self.detected_handle_position = None


    def get_info(self):
        '''Display various information about the robot arm'''
        # We can get the name of the reference frame for this robot:
        planning_frame = self.arm_group.get_planning_frame()
        rospy.logdebug("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.arm_group.get_end_effector_link()
        rospy.logdebug("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        rospy.logdebug("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        rospy.logdebug("============ Printing robot state")
        rospy.logdebug(self.robot.get_current_state())
        rospy.logdebug("")

        rospy.logdebug("Arm_pose:")
        rospy.logdebug(self.arm_group.get_current_pose())
        rospy.logdebug("Gripper joints:")
        rospy.logdebug(self.gripper_group.get_current_joint_values())

    def close_gripper(self):
        '''
        Close the gripper
        TODO:
        There seems to be a bug in the kinova driver (or some other part) that prevents this motion to ever finish, which in turn prevents other plans to execute.
        A workaround is to call gripper_group.stop() before executing another plan.
        '''
        joint_goal = [0.75, 0.75, 0.75]
        self.gripper_group.go(joint_goal, wait=True)

        #self.gripper_group.set_named_target("Close")
        #self.gripper_group.go(wait=True)

    def open_gripper(self):
        '''Open the gripper'''
        joint_goal = [0.0, 0.0, 0.0]
        self.gripper_group.go(joint_goal, wait=True)

        #self.gripper_group.set_named_target("Open")
        #self.gripper_group.go(wait=True)

    def plan_linear(self, start_pose, goal_pose, eef=0.01):
        '''
        Plan a linear motion from start_pose to goal_pose with interpolation points a maximum of eef meters apart.
        Returns the plan as well as the calculated waypoints.
        '''
        waypoints = []
        waypoints.append(start_pose)
        waypoints.append(goal_pose)

        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                eef,         # eef_step
                                5.00)        # jump_threshold

        return plan, waypoints

    def plan_arc(self, hinge_pose, start_pose, angle, interpolation_steps = 10, eef=0.01):
        '''
        Plan a cirular motion from start_pose around hinge_pose with the given angle.
        The arc is aproximated with interpolation_steps points with linear interpolation between these points a maximum of eef meters apart.
        Returns the plan as well as the calculated waypoints.
        '''
        # handle_pose = self.arm_group.get_current_pose().pose

        q = hinge_pose.orientation
        q = [q.x, q.y, q.z, q.w]
        transformed_x_axis = numpy.dot(quaternion_matrix(q), (1, 0, 0, 1))
        hinge_position = [hinge_pose.position.x, hinge_pose.position.y, hinge_pose.position.z]
        interpolation_matrix = rotation_matrix(angle / interpolation_steps, transformed_x_axis, hinge_position)

        #Calculate the waypoints based on these datapoints
        waypoints = []
        waypoints.append(start_pose)

        current_matrix = pose_to_matrix(start_pose)

        #Construct the path by interpolating it step by step
        for i in range(interpolation_steps):
            #Convert it to a matrix, transform it
            current_matrix = concatenate_matrices(interpolation_matrix, current_matrix)
            
            #and convert it back into a pose
            waypoints.append(matrix_to_pose(current_matrix))
            
        #Plan path
        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                eef,         # eef_step
                                5.00)        # jump_threshold

        return plan, waypoints

    def home_arm(self):
        rospy.logwarn("NOT IMPLEMENTED YET")
        # rosservice call /'${kinova_robotType}_driver'/in/home_arm
       

rc = RobotCommander()

def cmd_callback(data):
    '''Called once we send a signal manually to plan and move'''

    rc.arm_group.set_max_velocity_scaling_factor(0.5)
    # Fix a bug where the gripper movement would not finish which blocks executing new commands
    rc.gripper_group.stop()

    rospy.loginfo(data)

    if data.cmd == Command.SETGOAL:
        rospy.loginfo("Goal is set to handle position")
        rc.saved_handle_position = rc.detected_handle_position

    if data.cmd == Command.SETTESTGOAL:
        rospy.loginfo("Goal is set to test point")
        rc.saved_handle_position = PointStamped(
            Header(
                frame_id=rc.planning_frame,
                stamp=rospy.Time.now()
            ), Point(-0.5, -0.6, 0.02)
        )

    elif data.cmd == Command.MOVEWITHOFFSET_P2P:
        rospy.loginfo("Moving arm into position with offset")
        
        if rc.saved_handle_position is None:
            rospy.logerr("No door handle found")
            return

        start_pose = rc.arm_group.get_current_pose().pose

        #Grip from the front
        #q = quaternion_from_euler(-pi/2, pi, pi/2)
        q = quaternion_from_euler(-pi/2, pi, pi)
        goal_orientation = Quaternion(q[0],q[1],q[2],q[3])

        goal_pose = Pose(position = copy.deepcopy(rc.saved_handle_position.point), orientation = goal_orientation)
        goal_pose.position.y += 0.1

        rc.arm_group.set_pose_target(goal_pose)
        (success, plan, time, error) = rc.arm_group.plan()
        rc.execute_plans(plan)
    
    elif data.cmd == Command.MOVEWITHOFFSET_LIN:
        rospy.loginfo("Moving arm linearly into position with offset")
        if rc.detected_handle_position is None:
            rospy.logerr("No door handle found")
            return

        start_pose = rc.arm_group.get_current_pose().pose

        #Grip from the front
        q = quaternion_from_euler(-pi/2, pi, pi)
        goal_orientation = Quaternion(q[0],q[1],q[2],q[3])

        goal_pose = Pose(position = copy.deepcopy(rc.saved_handle_position.point), orientation = goal_orientation)
        goal_pose.position.y += 0.1

        plan, wps = rc.plan_linear(start_pose, goal_pose)
        rc.visualize_waypoints(wps)
        rc.wps = wps # save the waypoints so we can perform the reverse operation

        rc.execute_plans(plan)

    elif data.cmd == Command.MOVE:
        rospy.loginfo("Moving arm linearly into grasping position")
        if rc.detected_handle_position is None:
            rospy.logerr("No door handle found")
            return

        start_pose = rc.arm_group.get_current_pose().pose

        #Grip from the front
        q = quaternion_from_euler(-pi/2, pi, pi)
        goal_orientation = Quaternion(q[0],q[1],q[2],q[3])

        goal_pose = Pose( position = copy.deepcopy(rc.saved_handle_position.point), orientation=goal_orientation )
        #goal_pose.position.y += 0.0

        plan, wps = rc.plan_linear(start_pose, goal_pose)
        rc.visualize_waypoints(wps)
        rc.wps = wps # save the waypoints so we can perform the reverse operation

        rc.execute_plans(plan)

    elif data.cmd == Command.OPENGRIPPER:
        rospy.loginfo("Opening the grippre")
        rc.open_gripper()
    
    elif data.cmd == Command.CLOSEGRIPPER:
        rospy.loginfo("Closing the gripper")
        rc.close_gripper()

    elif data.cmd == Command.OPENDOOR:
        rospy.loginfo("Opening the door")

        #TODO: auto detect this
        handle_radius = 0.66 # hinge to handle

        handle_pose = rc.arm_group.get_current_pose().pose

        p = copy.deepcopy(handle_pose.position)
        p.x += handle_radius
        q = quaternion_from_euler(0,pi/2,0)
        hinge_pose = Pose(p, Quaternion(q[0],q[1],q[2],q[3]))

        plan, wps = rc.plan_arc(hinge_pose, handle_pose, pi / 6)
        rc.visualize_waypoints(wps)
        rc.wps = wps

        rc.execute_plans(plan)

    elif data.cmd == Command.REVERSE:
        rospy.loginfo("Reverse last cartesian movement")
        wps = rc.wps[::-1]  #reverse
        (plan, fraction) = rc.arm_group.compute_cartesian_path(
                                wps,    # waypoints to follow
                                0.01,   # eef_step
                                5.00)   # jump_threshold
        rc.visualize_waypoints(wps)
        rc.wps = wps

        rc.execute_plans(plan)


def pos_callback(data):
    '''Called every time we detect a handle'''

    #TODO: Try block?
    rc.detected_handle_position = tfBuffer.transform(data, "root", rospy.Duration(1.0))


def listener():
    '''
    Listen for target objects as move goal
    and commands to select the appropriate actions
    
    The target objects comes from camera_interface.py

    The commands are send manually by the user,
    but this will later be intergrated into hardware
    '''

    global tfBuffer, tfListener

    rospy.init_node('robot_interface', anonymous=True)
    #rate = rospy.Rate(10)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    rospy.loginfo("Starting robot listening")

    rospy.Subscriber('armchair/handle_position', PointStamped, pos_callback)
    rospy.Subscriber('armchair/cmd', Command, cmd_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
