#! /usr/bin/env bash

# We remove a folder that otherwise gives issues in ROS2 launches
sudo rm -r /home/user/.ros

# We set up the environment for ROS2
. /usr/share/gazebo/setup.sh
export arm_ws=$(pwd)
export GAZEBO_RESOURCE_PATH=$(arm_ws)/ur_descriptioin:${GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=$(arm_ws)/robotnik_sensors:${GAZEBO_MODEL_PATH}

ros2 launch office_gazebo empty_ur3e.launch.xml