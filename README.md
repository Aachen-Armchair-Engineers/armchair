 # armchair

## Requirements
  * ROS noetic
  * Kinova-ros for noetic
    * either our own or the official version (ported after we already did that ourselves)
  * Depthai-ros
  * Lots of package dependencies
  * Optional: Foxglove Studio for visualisation
 
## Install
  * Put everything your workspace/src and catkin_make
  
## Use
  * roslaunch armchair armchair.launch
      * starts the entire project
      * Optional arguments:
        * model: armchair (default), mobilenet, yolov4
        * virtual_robot: true (default), false
  * roslaunch armchair mobile_publisher.launch
      * Only starts the mobilenet part
            * Optional arguments:
              * model: armchair (default), mobilenet
