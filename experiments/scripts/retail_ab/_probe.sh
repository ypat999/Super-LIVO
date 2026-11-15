#!/usr/bin/env bash
# 15 s 连通性验证：Retail_Street_d2 播放 → /livox/imu + /livox/lidar 到达数
set +u
source /opt/ros/humble/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=86
python3 /home/ywj/git/Super-LIVO/third_party/Super-LIO/exp/_count.py &
CP=$!
sleep 2
timeout 15 ros2 bag play /home/ywj/slam_data/Retail_Street_d2 --topics /livox/imu /livox/lidar /left_camera/image > /tmp/rs_probe_play.log 2>&1
wait $CP
