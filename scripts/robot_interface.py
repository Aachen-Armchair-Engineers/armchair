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
last_pos = None

#TODO: Use this for the entire opening process
#See: http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html#displaying-a-trajectory
# https://answers.ros.org/question/278616/how-to-create-a-publisher-about-trajectory-path-then-show-it-in-rviz/
# http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
display_trajectory_publisher = rospy.Publisher( 
       "/move_group/display_planned_path",
       moveit_msgs.msg.DisplayTrajectory,
       queue_size=20,
    )

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


pub_hinge_handle = rospy.Publisher(
        '/armchair/pose_hinge_handle',
        PoseArray,
        queue_size=10
    )

pub_poses = rospy.Publisher(
        '/armchair/poses',
        PoseArray,
        queue_size=10
    )

class RobotCommander:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.arm_group =  moveit_commander.MoveGroupCommander("arm")
        self.gripper_group =  moveit_commander.MoveGroupCommander("gripper")

        self.arm_group.set_max_velocity_scaling_factor(0.3)

        self.planning_frame = self.arm_group.get_planning_frame()


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

    def plan_with_offset(self, offset = 0.0):
        global last_pos

        #no handle detected? do nothing
        if last_pos is None:
            return

        #Make sure we dont get any funky missed timing errors
        last_pos.header.stamp = rospy.Time.now()

        pose = self.arm_group.get_current_pose().pose
        pose.position = last_pos.point

        #TODO: make the offset direction based on the relation to the robot base/current TCP
        pose.position.y += offset
        
        #Grip from the front
        q = quaternion_from_euler(pi/2,pi/2,0)
        pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        
        rospy.loginfo(pose)

        self.arm_group.set_pose_target(pose)
        self.arm_group.plan()

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

    def plan_circular_motion(self):
        #TODO: How to cartesian a circle
        # https://answers.ros.org/question/211343/how-to-use-moveit-for-planning-circular-path-with-a-robotic-arm/
        # http://docs.ros.org/en/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#ad6b02d15000d5b17c89b15a0f744b47c
        # http://docs.ros.org/en/indigo/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html

        handle_radius = 0.5 # hinge to handle
        opening_angle = pi / 2
        interpolations = 10

        #Calculate the waypoints based on these datapoints
        waypoints = []

        # handle_pose = self.arm_group.get_current_pose().pose
        
        p = Point(-0.3, 0, 0.5)
        q = quaternion_from_euler(-pi/2, pi, pi/2)
        handle_pose = Pose(copy.deepcopy(p), Quaternion(q[0],q[1],q[2],q[3]))

        p.y -= handle_radius
        q = quaternion_from_euler(0,pi/2,0)
        hinge_pose = Pose(p, Quaternion(q[0],q[1],q[2],q[3]))

        #Visualize the hinge and handle poses:
        pub_hinge_handle.publish( 
                PoseArray(
                    Header(
                        frame_id='world',
                        stamp=rospy.Time.now()
                    ),
                    [handle_pose, hinge_pose]
                )
            )


        q = hinge_pose.orientation
        q = [q.x, q.y, q.z, q.w]
        transformed_x_axis = numpy.dot(quaternion_matrix(q), (1, 0, 0, 1))
        hinge_position = [hinge_pose.position.x, hinge_pose.position.y, hinge_pose.position.z]
        interpolation_matrix = rotation_matrix(opening_angle / interpolations, transformed_x_axis, hinge_position)

        #Construct the path by interpolating it step by step

        # start with the current pose
        #waypoints.append(self.arm_group.get_current_pose().pose)


        waypoints.append(handle_pose)

        handle_matrix = pose_to_matrix(handle_pose)

        for i in range(interpolations):
            #Convert it to a matrix, transform it
            handle_matrix = concatenate_matrices(interpolation_matrix, handle_matrix)
            
            #and convert it back into a pose
            waypoints.append(matrix_to_pose(handle_matrix))

        #Send the path as pose array to make degguing the motion easier
        pub_poses.publish( 
                PoseArray(
                    Header(
                        frame_id=self.planning_frame,
                        stamp=rospy.Time.now()
                    ),
                    waypoints
                )
            )

        #Plan path
        (plan3, fraction) = self.arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.00)        # jump_threshold

        #Diplay the trajectory as well
        #TODO: Plan & display the entire movement 
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan3)
        display_trajectory_publisher.publish(display_trajectory)

    def go(self):
        self.arm_group.go(wait=True)
        rospy.loginfo("finished")

    def robot_routine(self):
        '''
        Move swiftly right before the handle, then grip it slowly,
        follow up by a circular movement opening the door
        and move back into the neutral position
        '''

        #Move fast to the position before the handle
        self.arm_group.set_max_velocity_scaling_factor(0.3)

        self.plan_with_offset(0.2)
        self.go()


        #Open the gripper
        self.open_hand()


        #Slowly move the last distance
        self.arm_group.set_max_velocity_scaling_factor(0.1)

        self.plan_with_offset(0.1)
        self.go()


        #close the gripper 
        self.close_hand()

        
        # Move back to the previous position
        self.plan_with_offset(0.2)
        #TODO: use this instead:
        #robot_plan_circular_motion()
        self.go()


        #Open the gripper again
        self.open_hand()


        #Go back into neutral position
        self.arm_group.set_max_velocity_scaling_factor(0.3)

        self.plan_homing()
        self.go()

rc = RobotCommander()

def btn_callback(data):
    '''called once we send a signal manually to plan and move'''

    rospy.loginfo(data)
    if data.data == 0:
        rospy.loginfo("received plan signal for offset pos")
        rc.plan_with_offset(0.2)
    
    elif data.data == 1:
        rospy.loginfo("received plan signal for final pos")
        rc.plan_with_offset(0.1)

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
        rc.plan_circular_motion()

def pos_callback(data):
    '''called every time we detect an object'''

    global last_pos, tfBuffer
    last_pos = tfBuffer.transform(data, "root")


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
