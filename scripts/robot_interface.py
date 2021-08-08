#!/usr/bin/env python3
'''
This script gets selected data from the  camera and moves the robot accordingly:
camera_interface.py -> robot_interface.py -> Kinova Jaco
'''

from math import pi
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

tfBuffer = None
tfListener = None

#TODO: Use this for the entire opening process
#See: http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html#displaying-a-trajectory
# https://answers.ros.org/question/278616/how-to-create-a-publisher-about-trajectory-path-then-show-it-in-rviz/
# http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html

def pose_to_matrix(pose):
    p = pose.position
    p = [p.x, p.y, p.z]
    translation = translation_matrix(p)
    r = pose.orientation
    r = [r.x, r.y, r.z, r.w]
    rotation = quaternion_matrix(r)
    return concatenate_matrices(translation, rotation)

def matrix_to_pose(matrix):
    translation = translation_from_matrix(matrix)
    translation = Point(translation[0], translation[1], translation[2])
    rotation = quaternion_from_matrix(matrix)
    rotation = Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])

    return Pose(translation, rotation)

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


def flatten(t):
    #rospy.logerr("BBBBBBBBBBBBBBBBBBBBBBBBB")
    #rospy.logerr(f"Before: {t}")
    t = [item for sublist in t for item in sublist]
    #rospy.logerr(f"After: {t}")
    return t


class RobotCommander:
    '''TODO'''

    def visualize_plans(self, *plans):
        '''TODO'''
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()

        for plan in plans:
            display_trajectory.trajectory.append(plan)

        display_trajectory_publisher.publish(display_trajectory)

    def visualize_waypoints(self, *waypoints):
        '''Send the path as pose array to make debugging the motion easier'''

        rospy.logerr("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
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

    def open_hand(self):
        joint_goal = [0.5, 0.5, 0.5]
        self.gripper_group.go(joint_goal, wait=True)

    def close_hand(self):
        joint_goal = [0.0, 0.0, 0.0]
        self.gripper_group.go(joint_goal, wait=True)

    def plan_linear(self, start_pose, goal_pose, eef=0.01):
        waypoints = []
        waypoints.append(start_pose)
        waypoints.append(goal_pose)

        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                eef,         # eef_step
                                0.00)        # jump_threshold

        return plan, waypoints

    def plan_arc(self, hinge_pose, start_pose, angle, interpolation_steps = 10, eef=0.01):

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
                                0.00)        # jump_threshold

        return plan, waypoints

    def plan_homing(self):
        rospy.logwarn("NOT IMPLEMENTED YET")

        #rosservice call /'${kinova_robotType}_driver'/in/home_arm
        # rospy.wait_for_service("/j2n6s300/in/home_arm")
        # try:
        #    home = rospy.ServiceProxy("/j2n6s300/in/home_arm", HomeArm)
        #    home()
        #    return None
        # except rospy.ServiceException, e:
        #    print ("Service call failed: %s"%e)

    def routine(self):
        '''
        Move swiftly right before the handle, then grip it slowly,
        follow up by a circular movement opening the door
        and move back into the neutral position
        '''

        plans = []
        wps = []

        #Move fast to the position before the handle
        self.arm_group.set_max_velocity_scaling_factor(0.3)

        if self.detected_handle_position is None:
            rospy.logwarn("No door handle found")
        
        #Use fixed test point here anyways
        self.detected_handle_position = PointStamped(
                Header(
                    frame_id=self.planning_frame,
                    stamp=rospy.Time.now()
                ), Point(-0.3, 0, 0.5)
        )


        rospy.loginfo("received plan signal for offset pos")
        start_pose = rc.arm_group.get_current_pose().pose

        #Grip from the front
        q = quaternion_from_euler(-pi/2, pi, pi/2)
        goal_orientation = Quaternion(q[0],q[1],q[2],q[3])

        goal_pose = Pose( position = self.detected_handle_position.point, orientation=goal_orientation )
        goal_pose.position.y += 0.1

        plan, wp = rc.plan_linear(start_pose, goal_pose)
        plans.append(plan)
        wps.append(wp)

        #The second movement
        goal_pose = Pose( position = self.detected_handle_position.point, orientation=goal_orientation )
        goal_pose.position.y += 0.0

        plan, wp = rc.plan_linear(wp[-1], goal_pose)
        plans.append(plan)
        wps.append(wp)

        #Plan circular, door-opening motion        
        handle_radius = 0.5 # hinge to handle

        p = self.detected_handle_position.point
        q = quaternion_from_euler(-pi/2, pi, pi/2)
        handle_pose = Pose(copy.deepcopy(p), Quaternion(q[0],q[1],q[2],q[3]))

        p.y -= handle_radius
        q = quaternion_from_euler(0,pi/2,0)
        hinge_pose = Pose(p, Quaternion(q[0],q[1],q[2],q[3]))

        plan, wp = rc.plan_arc(hinge_pose, handle_pose, pi / 2)
        plans.append(plan)
        wps.append(wp)

        #Go back to the start position:
        plan, wp = rc.plan_linear(wp[-1], wp[0])
        plans.append(plan)
        wps.append(wp)

        #show all the waypoints in rvit
        rc.visualize_waypoints(flatten(wps))


        #Move
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        rc.execute_plans(plans[0])

        #Open the gripper
        self.open_hand()

        self.arm_group.set_max_velocity_scaling_factor(0.1)
        rc.execute_plans(plans[1])

        #close the gripper 
        self.close_hand()


        rc.execute_plans(plans[2])
        
        #Open the gripper again
        self.open_hand()
        
        rc.execute_plans(plans[3])

        self.arm_group.set_max_velocity_scaling_factor(0.3)


rc = RobotCommander()

def btn_callback(data):
    '''called once we send a signal manually to plan and move'''

    rospy.loginfo(data)
    if data.data == 0:
        if rc.detected_handle_position is None:
            rospy.logwarn("No door handle found")
            return

        rospy.loginfo("received plan signal for offset pos")
        start_pose = rc.arm_group.get_current_pose().pose

        #Grip from the front
        q = quaternion_from_euler(-pi/2, pi, pi/2)
        goal_orientation = Quaternion(q[0],q[1],q[2],q[3])

        goal_pose = Pose( position = rc.detected_handle_position.point, orientation=goal_orientation )
        goal_pose.position.y += 0.1

        plan, wps = rc.plan_linear(start_pose, goal_pose)

    
    elif data.data == 1:
        rospy.logerr("WIP")

        #rospy.loginfo("received plan signal for final pos")
        #rc.plan_with_offset(0.1)

    elif data.data == 2:
        rospy.logerr("WIP")
        rc.plan_homing()

    elif data.data == 3:
        rospy.loginfo("received go signal")
        rc.go()
    
    elif data.data == 10:
        rospy.loginfo("opening")
        rc.open_hand()
    
    elif data.data == 11:
        rospy.loginfo("closing")
        rc.close_hand()

    elif data.data == 20:
        rospy.logerr("WIP")
        rc.routine()

    elif data.data == 30:
        rospy.logerr("WIP")
    
        handle_radius = 0.5 # hinge to handle

        p = Point(-0.5, 0.2, 0.5)
        q = quaternion_from_euler(-pi/2, pi, pi/2)
        handle_pose = Pose(copy.deepcopy(p), Quaternion(q[0],q[1],q[2],q[3]))

        p.y -= handle_radius
        q = quaternion_from_euler(0,pi/2,0)
        hinge_pose = Pose(p, Quaternion(q[0],q[1],q[2],q[3]))

        plan, wps = rc.plan_arc(hinge_pose, handle_pose, pi / 2)

        rc.visualize_waypoints(wps)

def pos_callback(data):
    '''called every time we detect an object'''

    global tfBuffer
    #TODO: Try block?
    #rc.detected_handle_position = tfBuffer.transform(data, "root", rospy.Duration(1.0))


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
    rospy.Subscriber('armchair/cmd', Int32, btn_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
