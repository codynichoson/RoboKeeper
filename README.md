# RoboKeeper!
## Jonny Bosnich, Joshua Cho, Lio Liang, Marco Morales, Cody Nichoson 
****
## Note
edit `custom_ws/src/hdt_6dof_a24_pincer/hdt_6dof_a24_pincer_moveit/config/joint_limits.yaml` to make max velocities 2.5 instead of 0.785

edit `custom_ws/src/hdt_6dof_a24_pincer/hdt_6dof_a24_pincer_moveit/config/kinematics.yaml` and make the kinmeatic solver `lma_kinematics_plugin/LMAKinematicsPlugin`

 edit `hdt_6dof_a24_pincer/hdt_6dof_a24_pincer_description/urdf/hdt_arm.xacro` to make `pos_limit` equal to `"${8*M_PI}"` (line 4) 
## Overview
The Robokeeper ROS Noetic package allows HDT Adroit 6-dof manipulator arm to play goalkeeper.

## Dependencies


## Quickstart Guide
0. Install ROS Noetic on Ubuntu 20.04
1. Create catkin workspace

    ```
    $ source /opt/ros/noetic/setup.bash
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/
    $ catkin_make
    ```
2. Copy this repository into `src` folder
    ```
    $ cd ~/catkin_ws/src
    $ git clone git@github.com:ME495-EmbeddedSystems/final-project-robokeeper.git
    ```
3. Install required packages and build
    ```
    $ source devel/setup.bash
    $ rosdep install --from-paths src --ignore-src -r -y
    $ catkin_make
    ```
****
## Nodes and Launchfiles
### robokeeper_go.launch
This is the main launchfile used to operate robokeeper. It starts by launching `robokeeper_moveit.launch` which loads the necessary urdf file and hardware configuration, as well as the main `MoveIt!` executable. It then launches `intel_cam.launch` which starts the Intel Realsense camera. It also starts a `transforms` node which handles the calculation of transformation between various frames within the world. Finally, the launchfile starts a `motion_control` node that publishes appropriate joint state messages to actuate the arm. 


### robokeeper_moveit.launch
This launchfile loads robot description for the Adroit 6-dof manipulator arm, as well as its hardware and controller configuration from the `hdt_6dof_a24_pincer_description` package. It also includes `move_group.launch` from the `hdt_6dof_a24_pincer_moveit` package, which starts the move group that `MoveIt!` uses to plan the motion of the arm.

### intel_cam.launch
This launchfile starts the Intel Realsense camera by launching `rs_camera.launch` from the `realsense2_camera` package. It then launches `AprilTag_detection.launch` for AprilTag integration.

### AprilTag_detection.launch
This launchfile loads parameters necessary for integrating AprilTag detection, which is crucial for detecting the position of the robot relative to the camera. It starts `apriltag_ros_continuous_node` from the `apriltag_ros` package.

### motion_control
This node starts

### motion_planning

### perception

### transforms

## System Architecture

