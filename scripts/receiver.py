#!/usr/bin/env python

import rospy
from std_msgs.msg import String
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

def callback(data):
    for detection in data.detections:
        rospy.loginfo("%s (%2f, %2f, %2f)", labels[detection.results[0].id], detection.position.x, detection.position.y, detection.position.z)

def listener():
    rospy.init_node('receiver', anonymous=True)

    rospy.Subscriber("/yolov4_publisher/color/yolov4_Spatial_detections", SpatialDetectionArray, callback)
    #rospy.Subscriber("/yolov4_publisher/color/yolov4_Spatial_detections", String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
