#!/bin/bash

# Source ROS 2 setup files
source /opt/ros/foxy/setup.bash
source /opt/kobuki/install/setup.bash
export ROS_LOCALHOST_ONLY=1

# Run your ROS node
ros2 run uds_kobuki_ros uds_kobuki_ros --ros-args -p ip_address:="127.0.0.1"
