 # armchair

TODO: Add short project description and links to the teams here

## Requirements
  * ROS noetic
  * Kinova-ros **for noetic**
    * either our [fork](https://github.com/Aachen-Armchair-Engineers/kinova-ros) we ported or the [official version](https://github.com/Kinovarobotics/kinova-ros/tree/noetic-devel) (noetic branch was released after we already finished porting it ourselves)
  * [depthai-ros (gen2)](https://github.com/luxonis/depthai-ros/tree/noetic-devel)
  * Probably lots of package dependencies we forgot
  * Optional: [Foxglove Studio](https://foxglove.dev/) for visualisation
 
## Install
  * Put everything your workspace/src and catkin_make
  
## Use
* `roslaunch armchair armchair.launch`
  * starts the entire project
   * Optional arguments:
     * model: armchair (default), mobilenet, yolov4
     * virtual_robot: true (default), false
 * `roslaunch armchair mobile_publisher.launch`
   * Only starts the mobilenet part
   * Optional arguments:
     * model: armchair (default), mobilenet
