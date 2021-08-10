#!/usr/bin/env python3
'''
This script sits inbetween the detection and robot control:
mobilenet_publisher.cpp -> camera_interface.py -> robot_interface.py

'''

import rospy
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion, Vector3
from depthai_ros_msgs.msg import SpatialDetectionArray
from tf.transformations import quaternion_from_euler

from armchair.msg import DoorInfo

#For visualizing in rviz and foxglove
#See: https://foxglove.dev/blog/annotate-your-robots-camera-images-with-image-markers
from visualization_msgs.msg import Marker, MarkerArray, ImageMarker
from foxglove_msgs.msg import ImageMarkerArray

#TODO: Crop and visualize the depth image in rviz:
#See: https://gist.github.com/bhaskara/2400165
#from sensor_msgs.msg import PointCloud2

labels = []
target_label = ""

def init_labels(model):
    ''' Set the labels according to the model used '''
    global labels, target_label

    if model == 'armchair':
        labels = ["", "door", "handle", "cabinet door", "refridgerator door"]
        target_label = "handle"

    elif model == 'mobilenet':
        labels = [
            "background", "aeroplane", "bicycle", "bird", "boat",
            "bottle", "bus", "car", "cat", "chair",
            "cow", "diningtable", "dog", "horse", "motorbike",
            "person", "pottedplant", "sheep", "sofa", "train",
            "tvmonitor"
        ]
        target_label = "bottle"

    elif model == 'yolov4':
        labels = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train",
                 "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
                 "bird", "cat", "dog", "horse", "sheep", "cow", "elephant",
                 "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie",
                 "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
                 "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
                 "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
                 "orange", "broccoli",   "carrot", "hot dog", "pizza", "donut", "cake",
                 "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor",
                 "laptop", "mouse", "remote", "keyboard", "cell phone",  "microwave", "oven",
                 "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
                 "teddy bear", "hair drier", "toothbrush"]
        target_label = "cup"

    else:
        rospy.logerr("Invalid neuronal network selected, aborting")

pub_handle = rospy.Publisher(
        '/armchair/handle_position',
        PointStamped,
        queue_size=10
    )

pub_door_info = rospy.Publisher(
        '/armchair/door_info',
        DoorInfo, #TODO: Custom datatype
        queue_size=10
    )


pub_rviz_markers = rospy.Publisher(
        '/mobilenet_node_custom/color/rviz_markers',
        MarkerArray,
        queue_size=1
    )

pub_foxglove_markers = rospy.Publisher(
        '/mobilenet_node_custom/color/foxglove_markers',
        ImageMarkerArray,
        queue_size=1
    )

def callback(data):
    '''
    If one or mor e targeted objects are found, pulish it to
    /armchair/handle_position

    This will be visualized in rviz by a pink sphere
    and is the position the robot will assume the handle to be.

    Furthermore publish all detected bounding boxes and colour them:
    Red: Target object(s)
    Cyan: Normal object(s)

    We use Foxglove Studio to visualize this
    '''


    #Find best detection
    #Alternatively use the clostest or manually select one
    handles = list(filter(lambda l : labels[l.results[0].id] == target_label, data.detections))

    if handles:
        best_match = max(handles, key=lambda h: h.results[0].score)
        pub_handle.publish(
            PointStamped(
                Header(
                    stamp = rospy.Time.now(),
                    frame_id = 'oak-d_frame'
                ),
                #Swap the axis to make the transform in ros easier
                Point(
                    best_match.position.x,
                    best_match.position.z,
                    best_match.position.y
                )
            )
        )

        #If we run the door detection publish additional information
        if target_label == "handle":
            doors = list( filter(lambda l : labels[l.results[0].id] not in ["", "handle"] , data.detections) )
            if doors:
                analyse_handle_and_door(best_match, doors)


    #Create markers for all relevant object
    normal_markers = MarkerArray()
    image_markers = ImageMarkerArray()

    for detection in data.detections:
        #Calculate the positions of the bounding box edges
        vertex_left = detection.bbox.center.x - detection.bbox.size_x/2
        vertex_right = detection.bbox.center.x + detection.bbox.size_x/2
        vertex_top = detection.bbox.center.y - detection.bbox.size_y/2
        vertex_bot = detection.bbox.center.y + detection.bbox.size_y/2

        #Scale from 320x320 square coordinates to camera image (1280*720) coordinates
        vertex_left *= 1280/320
        vertex_right *= 1280/320
        vertex_top *= 720/320
        vertex_bot *= 720/320


        #Highlight the BBoxes of the targets
        label = labels[detection.results[0].id]
        if label == target_label:
            marker_color=ColorRGBA(255, 0, 0, 1)
        #TODO: Add more labels of special interest and (multiple) different colours for these
        # elif label in highlight_lables.labels:
        #    color = highlight_labels.colour[label]
        #Colour for normal bboxes
        else:
            marker_color=ColorRGBA(0, 255, 255, 1)

        normal_markers.markers.append(
            Marker(
                header = Header(
                    stamp = rospy.Time.now(),
                    frame_id = 'oak-d_frame'
                ),
                id=len(normal_markers.markers),
                ns='/armchair/rviz_markers',
                type=2,
                action=0,
                pose=Pose(
                    Point(
                        detection.position.x,
                        detection.position.z,
                        detection.position.y
                    ),
                    Quaternion(0,0,0,1)
                ),
                scale=Vector3(0.05, 0.05, 0.05),
                color=marker_color,
                #If we dont let them decay we need to clean up after them
                lifetime=rospy.Duration(0.3)
            )
        )

        image_markers.markers.append(
            ImageMarker(
                header = Header(
                    stamp = rospy.Time.now(),
                    frame_id = 'oak-d_frame'
                ),
                type=ImageMarker.POLYGON,
                outline_color=marker_color,
                points=[
                    Point(vertex_left, vertex_top, 0),
                    Point(vertex_right, vertex_top, 0),
                    Point(vertex_right, vertex_bot, 0),
                    Point(vertex_left, vertex_bot, 0),
                ],
            )
        )

    pub_rviz_markers.publish(normal_markers)
    pub_foxglove_markers.publish(image_markers)

    rospy.logdebug('-------')

def analyse_handle_and_door(handle, doors):
    '''
    TODO:
     - This assumes that the bounding boxes match perfectly
     - visualize information somewhere for easier debugging
     - no pressing down the handle or not descission yet
     - Plane detection for non-perpendicular door normals
    '''
    rospy.logerr('Not working stable yet')
    return

    #Check if handle is inside a door bounding_box
    doors = list( filter(
        lambda l :
            l.bbox.center.x - l.bbox.size_x/2  < handle.bbox.center.x <  l.bbox.center.x + l.bbox.size_x/2 and 
            l.bbox.center.y - l.bbox.size_y/2  < handle.bbox.center.y <  l.bbox.center.y + l.bbox.size_y/2 ,
            doors) )

    #Assume theres only one valid door at max:
    if not doors:
        return
    
    door = doors[0]


    #Handle orientation (horizontal, vertical)
    if 0.6 * handle.bbox.size_x > handle.bbox.size_y:
        rospy.loginfo('horizontal')
        _handle_orientaton = DoorInfo.HORIZONTAL

    if 0.6 * handle.bbox.size_y > handle.bbox.size_x:
        rospy.loginfo('vertical')
        _handle_orientaton = DoorInfo.VERTICAL

    else:
        rospy.loginfo('orientation undecidabele')
        _handle_orientaton = DoorInfo.UNDEFINED


    #Check which side the handle is closest too
    #Hinge is always on the edge further away

    if (door.bbox.center.x - handle.bbox.center.x) / door.bbox.size_x > 0.60:
        rospy.logerr('left')
        _handle_side = DoorInfo.LEFT

        hinge_position = Point(handle.position.x - 2 * door.position.x, handle.position.y - 2 * door.position.y, door.position.z)
    elif (door.bbox.center.x - handle.bbox.center.x) / door.bbox.size_x < 1.00- 0.60:
        rospy.logerr('right')
        _handle_side = DoorInfo.RIGHT

        hinge_position = Point(handle.position.x + 2 * door.position.x, handle.position.y + 2 * door.position.y, door.position.z)

    elif (door.bbox.center.y - handle.bbox.center.y) / door.bbox.size_y < 1.00- 0.60:
        rospy.logerr('down')
        _handle_side = DoorInfo.DOWN

        rospy.logwarn('Vertical hinges not implemented yet')

    else:
        rospy.loginfo('relative handle position unclear')
        _handle_side = DoorInfo.UNDEFINED


    #distance between handle and side thats further away -> radius (not needed explicitly)

    #Hinge pose    
    _hinge_pose=Pose(
        hinge_position,
        quaternion_from_euler(0,pi/2,0)
    ),


    #Handle pose
    _handle_pose=Pose(
        handle.position,
        quaternion_from_euler(-pi/2, pi, pi)

    ),


    pub_door_info.publish(
        DoorInfo(
            header = Header(
                stamp = rospy.Time.now(),
                frame_id = 'root'
            ),
            hinge_pose = _hinge_pose,
            handle_pose = _handle_pose,
            handle_orientation = _handle_orientaton,
            handle_side = DoorInfo.RIGHT,
        )
    )
    

def listener():
    '''
    Listen for spatial detections, filter them
    and if an object of interest is found
    pass the position along to the robot controller
    '''

    rospy.init_node('camera_interface', anonymous=True)

    #Set in mobile_publisher.launch
    model = rospy.get_param('/camera_interface/model')

    #Use the matching labels
    init_labels(model)

    #And the matching topic
    if model == 'yolov4':
        topic = '/yolov4_publisher/color/yolov4_Spatial_detections'
    else:
        topic = '/mobilenet_node_custom/color/spatial_detections'

    rospy.Subscriber(
            topic,
            SpatialDetectionArray,
            callback
        )

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
