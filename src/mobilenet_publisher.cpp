//FIXME:
/*
#ifndef BLOB_PATH
#define BLOB_PATH "../resources/armchair.blob"
#endif
*/

//TODO: Use launch option?
#if 0
#define BLOB_PATH "/home/felix/ros_ws/src/armchair/resources/armchair.blob"
#define WIDTH 320
#define HEIGHT 320
#else
#define BLOB_PATH "/home/felix/ros_ws/src/armchair/resources/mobilenet-ssd_openvino_2021.2_6shave.blob"
#define WIDTH 300
#define HEIGHT 300
#endif

#include "ros/ros.h"

#include <iostream>
#include <cstdio>

//#include <depthai_examples/nn_pipeline.hpp>
#include <armchair/nn_pipeline.hpp>

#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <vision_msgs/Detection2DArray.h>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImgDetectionConverter.hpp>

//#include <depthai_bridge/SpatialImageConverter.hpp>
#include <depthai_bridge/SpatialDetectionConverter.hpp>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


int main(int argc, char** argv){

    ros::init(argc, argv, "mobilenet_node_custom");
    ros::NodeHandle pnh("~");
    
    std::string deviceName;
    std::string camera_param_uri;
    std::string nnPath(BLOB_PATH);

    ROS_INFO("init");
    ROS_INFO("Using nn at path %s", nnPath.c_str());
    
    int bad_params = 0;

    bad_params += !pnh.getParam("camera_name", deviceName);
    bad_params += !pnh.getParam("camera_param_uri", camera_param_uri);

    if (bad_params > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    DoorDetectionPipeline detectionPipeline;
    detectionPipeline.initDepthaiDev(nnPath, WIDTH, HEIGHT);
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = detectionPipeline.getExposedImageStreams();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> nNetDataQueues = detectionPipeline.getExposedNnetStreams();;

    std::string color_uri = camera_param_uri + "/" + "color.yaml";
    std::string stereo_uri = camera_param_uri + "/" + "right.yaml";

    dai::rosBridge::ImageConverter rgbConverter(deviceName + "_rgb_camera_optical_frame", false);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(imageDataQueues[0],
                                                                                     pnh, 
                                                                                     std::string("color/image"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rgbConverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     color_uri,
                                                                                     "color");

/*
    dai::rosBridge::ImgDetectionConverter detConverter(deviceName + "_rgb_camera_optical_frame", 320, 320, false);
    dai::rosBridge::BridgePublisher<vision_msgs::Detection2DArray, dai::ImgDetections> detectionPublish(nNetDataQueues[0],
                                                                                                         pnh, 
                                                                                                         std::string("color/mobilenet_detections"),
                                                                                                         std::bind(static_cast<void(dai::rosBridge::ImgDetectionConverter::*)(std::shared_ptr<dai::ImgDetections>, 
                                                                                                         vision_msgs::Detection2DArray&)>(&dai::rosBridge::ImgDetectionConverter::toRosMsg), 
                                                                                                         &detConverter,
                                                                                                         std::placeholders::_1, 
                                                                                                         std::placeholders::_2) , 
                                                                                                         30);
*/

    dai::rosBridge::SpatialDetectionConverter detConverter(deviceName + "_rgb_camera_optical_frame", WIDTH, HEIGHT, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(nNetDataQueues[0],
                                                                                                         pnh, 
                                                                                                         std::string("color/mobilenet_detections"),
                                                                                                         std::bind(static_cast<void(dai::rosBridge::SpatialDetectionConverter::*)(std::shared_ptr<dai::SpatialImgDetections>, 
                                                                                                         depthai_ros_msgs::SpatialDetectionArray&)>(&dai::rosBridge::SpatialDetectionConverter::toRosMsg), 
                                                                                                         &detConverter,
                                                                                                         std::placeholders::_1, 
                                                                                                         std::placeholders::_2) , 
                                                                                                         30);
    
    dai::rosBridge::ImageConverter depthConverter(deviceName + "_right_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(imageDataQueues[1],
                                                                                     pnh, 
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &depthConverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     stereo_uri,
                                                                                     "stereo");


    detectionPublish.startPublisherThread();
    rgbPublish.addPubisherCallback(); // addPubisherCallback works only when the dataqueue is non blocking.
    depthPublish.addPubisherCallback();


    ROS_INFO("SPINNING");


    ros::spin();

    return 0;
}

