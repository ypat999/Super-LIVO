#!/usr/bin/env bash
set -o pipefail
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
source /home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/install/setup.bash
cd /home/ywj/slam_data/pl_ros2_ws
colcon build --packages-select point_lio 2>&1 | tail -25
echo EXIT=$?
