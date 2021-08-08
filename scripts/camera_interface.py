#!/usr/bin/env python3
'''
This script sits inbetween the detection and robot control:
mobilenet_publisher.cpp -> camera_interface.py -> robot_interface.py

'''

import rospy
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, PointStamped
from depthai_ros_msgs.msg import SpatialDetectionArray

#For visualizing in foxglove
#See: https://foxglove.dev/blog/annotate-your-robots-camera-images-with-image-markers
from visualization_msgs.msg import ImageMarker
from foxglove_msgs.msg import ImageMarkerArray

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

pub_markers = rospy.Publisher(
        '/mobilenet_node_custom/color/markers',
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

    highest_score = 0.0
    point_stamped = PointStamped()
    markers = ImageMarkerArray()

    #TODO: figure out a more pythonic approach for the whole loop
    for detection in data.detections:

        #Find best object
        #Alternatively use the clostest or manually select one
        if labels[detection.results[0].id] == target_label:
            rospy.logdebug('%s (%2f, %2f, %2f)',
                    labels[detection.results[0].id],
                    detection.position.x,
                    detection.position.y,
                    detection.position.z
                )

            if detection.results[0].score > highest_score:
                point_stamped.header.stamp = rospy.Time.now()
                point_stamped.header.frame_id = 'oak-d_frame'
                point_stamped.point = Point(
                        detection.position.x,
                        detection.position.z,
                        detection.position.y
                    )
                highest_score = detection.results[0].score


        #Create markers for all relevant object

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

        #Colour for normal bboxes

        #Highlight the BBoxes of the targets
        label = labels[detection.results[0].id]
        if label == target_label:
            color=ColorRGBA(255, 0, 0, 1)

        #TODO: Add more labels of special interest and (multiple) different colours for these
        #elif label in highlight_lables.labels:
        #    color = highlight_labels.colour[label]

        else:
            color=ColorRGBA(0, 255, 255, 1)

        markers.markers.append(
            ImageMarker(
                header=Header(),
                type=ImageMarker.POLYGON,
                outline_color=color,
                points=[
                    Point(vertex_left, vertex_top, 0),
                    Point(vertex_right, vertex_top, 0),
                    Point(vertex_right, vertex_bot, 0),
                    Point(vertex_left, vertex_bot, 0),
                ],
            )
        )

    if highest_score > 0.0:
        pub_handle.publish(point_stamped)

    pub_markers.publish(markers)

    rospy.logdebug('-------')

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
