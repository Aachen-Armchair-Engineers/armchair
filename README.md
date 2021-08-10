 # armchair

In this project we focus on the problem of opening doors, a difficult task for wheelchair users without assistance by third parties. We are integrating a robotic arm and a computer vision enabled camera system into the wheelchair. We use an OAK-D camera with a self-trained neural network to recognize the large number of commercially available door handles and find their position relative to the robot arm. The wheelchair will check if there is enough space to position itself, grip the handle and open the door while driving backwards at the same time. Closing the door is also intended to work in the same manner.

## Requirements
  * ROS noetic
  * Kinova-ros **for noetic**
    * either our [fork](https://github.com/Aachen-Armchair-Engineers/kinova-ros) we ported or the [official version](https://github.com/Kinovarobotics/kinova-ros/tree/noetic-devel) (noetic branch was released after we already finished porting it ourselves)
  * [depthai-ros (gen2)](https://github.com/luxonis/depthai-ros/tree/noetic-devel)
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
