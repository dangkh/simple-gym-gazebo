#!/bin/bash
# create catkin
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/dangkh/dang_gazebo
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_gazebo_plugin
cd ~/catkin_ws && catkin_make
cd src
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PWD/dang_gazebo/models:$PWD/turtlebot3_gazebo_plugin/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$PWD/turtlebot3_gazebo_plugin/build
cd 
git clone https://github.com/dangkh/simple-gym-gazebo
cd simple-gym-gazebo/tb3_description
cp turtlebot3_waffle.gazebo.xacro ~/catkin_ws/src/turtlebot3/turtlebot3_description/urdf
cp turtlebot3_waffle.urdf.xacro ~/catkin_ws/src/turtlebot3/turtlebot3_description/urdf