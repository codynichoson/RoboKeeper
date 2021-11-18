# RoboKeeper!
## Jonny Bosnich, Joshua Cho, Lio Liang, Marco Morales, Cody Nichoson 
****

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

