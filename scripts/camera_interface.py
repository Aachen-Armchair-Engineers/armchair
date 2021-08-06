#!/usr/bin/env python3

from numpy.lib.type_check import mintypecode
import rospy, tf2_ros
from std_msgs.msg import String, Header
from geometry_msgs.msg import Point, PointStamped
from depthai_ros_msgs.msg import SpatialDetectionArray

#For visualizing in foxglove
#See: https://foxglove.dev/blog/annotate-your-robots-camera-images-with-image-markers
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import ImageMarker
from foxglove_msgs.msg import ImageMarkerArray

labels = []
target_label = ""

def init_labels(model):
    global labels, target_label


    if model == 'armchair':
        labels = ["", "door", "handle", "cabinet door", "refridgerator door"]
        target_label = "handle"

    elif model == 'mobilenet':
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

    elif model == 'yolov4':
        labels = [
            "background",
            "aeroplane",
            "bicycle",
            "bird",
            "boat",
            "bottle",
            "bus",
            "car",
            "cat",
            "chair",
            "cow",
            "diningtable",
            "dog",
            "horse",
            "motorbike",
            "person",
            "pottedplant",
            "sheep",
            "sofa",
            "train",
            "tvmonitor"
        ]
        target_label = "bottle"

    else:
        rospy.logerr("Invalid neuronal network selected, aborting")
        exit(0)

pub_handle = rospy.Publisher('/armchair/handle_position', PointStamped, queue_size=10)
pub_markers = rospy.Publisher("/mobilenet_node_custom/color/markers", ImageMarkerArray, queue_size=1)

def callback(data):

    highest_score = 0.0
    point_stamped = PointStamped()
    markers = ImageMarkerArray()

    #TODO: figure out a more pythonic approach for the whole loop
    for d in data.detections:
        #Find best object
        #Alternatively use the clostest or manually select one
        if labels[d.results[0].id] == target_label:
            rospy.logdebug("%s (%2f, %2f, %2f)", labels[d.results[0].id], d.position.x, d.position.y, d.position.z)

            if d.results[0].score > highest_score:
                point_stamped.header.stamp = rospy.Time.now()
                point_stamped.header.frame_id = "oak-d_frame"
                point_stamped.point = Point(d.position.x, d.position.z, d.position.y)
                highest_score = d.results[0].score
        

        #Create markers for all relevant object
        
        #Calculate the positions of the bounding box edges
        vertex_left = d.bbox.center.x - d.bbox.size_x/2
        vertex_right = d.bbox.center.x + d.bbox.size_x/2
        vertex_top = d.bbox.center.y - d.bbox.size_y/2
        vertex_bot = d.bbox.center.y + d.bbox.size_y/2

        #Scale from 320x320 square coordinates to camera image (1280*720) coordinates 
        vertex_left *= 1280/320
        vertex_right *= 1280/320
        vertex_top *= 720/320
        vertex_bot *= 720/320

        #FIXME: Assigning the values in the constructor caused issues before, see if that is fixable
        color=ColorRGBA()
        color.r=0
        color.g=255
        color.b=255
        color.a=1

        #Highlight the BBoxes of the targets
        if labels[d.results[0].id] == target_label:
            color.r=255
            color.g=0
            color.b=0
            color.a=1

        #TODO: Add more labels of special interest and (multiple) different colours for these

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
            
    rospy.logdebug("-------")
    #rate.sleep()

def listener():
    rospy.init_node('camera_interface', anonymous=True)
    rate = rospy.Rate(10)

    model = rospy.get_param("/camera_interface/model")

    #Use the matching labels
    init_labels(model)

    #And the matching topic
    if model == "yolov4":
        rospy.Subscriber("/yolov4_publisher/color/yolov4_Spatial_detections", SpatialDetectionArray, callback)
    else:
        rospy.Subscriber("/mobilenet_node_custom/color/spatial_detections", SpatialDetectionArray, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

