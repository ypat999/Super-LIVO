#!/usr/bin/env bash
cd /home/ywj/git/Super-LIVO/third_party/Super-LIO || exit 1
source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
colcon build --packages-select super_lio 2>&1 | tail -20
echo "BUILD_EXIT=${PIPESTATUS[0]}"
