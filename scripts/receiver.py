#!/usr/bin/env python

import rospy, tf2_ros
from std_msgs.msg import String, Header
from geometry_msgs.msg import Point, PointStamped
from depthai_ros_msgs.msg import SpatialDetectionArray

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

pub = rospy.Publisher('armchair/door_position', PointStamped, queue_size=10)

def callback(data):
    for detection in data.detections:
        if detection.results[0].id == 0:
            rospy.loginfo("%s (%2f, %2f, %2f)", labels[detection.results[0].id], detection.position.x, detection.position.y, detection.position.z)
            point_stamped = PointStamped()
            point_stamped.header.stamp = rospy.Time.now()
            point_stamped.header.frame_id = "oak-d_frame"
            point_stamped.point = Point(detection.position.x, detection.position.y, detection.position.z)
            pub.publish(point_stamped)
            #rate.sleep()

def listener():
    rospy.init_node('receiver', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber("/yolov4_publisher/color/yolov4_Spatial_detections", SpatialDetectionArray, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

