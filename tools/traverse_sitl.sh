#!/bin/bash

source ~/catkin_ws/devel/setup.bash
# read from argument
PX4_HOME=$1
# check if argument is empty
if [ -z "$PX4_HOME" ]; then
    echo "Usage: $0 <px4_home>"
    exit 1
fi
# check if argument is a directory
if [ ! -d "$PX4_HOME" ]; then
    echo "Error: $PX4_HOME is not a directory"
    exit 1
fi
# delete trailing slash
PX4_HOME=${PX4_HOME%/}
# copied from https://docs.px4.io/main/en/simulation/ros_interface.html
source $PX4_HOME/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4_HOME $PX4_HOME/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_HOME
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_HOME/Tools/simulation/gazebo-classic/sitl_gazebo-classic

roslaunch sam_fp traverse_sitl.launch $2