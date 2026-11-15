#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash >/dev/null 2>&1
echo "===== official_vio_20261114/bag ====="
ros2 bag info /home/ywj/slam_data/official_vio_20261114/bag | sed -n '1,40p'
echo "===== Retail_Street ====="
ros2 bag info /home/ywj/slam_data/Retail_Street | sed -n '1,40p'
