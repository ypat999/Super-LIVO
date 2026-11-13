#!/usr/bin/env bash
set -e
mkdir -p /home/ywj/slam_data/livo_ws/src
cd /home/ywj/slam_data/livo_ws
rm -rf src/POINT-LIVO src/Point-line-LIVO
git clone --depth 1 https://github.com/Haz25/POINT-LIVO.git src/POINT-LIVO 2>&1 | tail -1
git clone --depth 1 -b v2.x https://github.com/S-Torin/Point-line-LIVO.git src/Point-line-LIVO 2>&1 | tail -1
for P in src/POINT-LIVO src/Point-line-LIVO; do
  echo "########## $P ##########"
  ls "$P"
  echo "--- package.xml ---"
  find "$P" -maxdepth 2 -name package.xml -exec grep -H -E "<name>|ament_cmake|catkin|livox" {} \;
  echo "--- configs ---"
  find "$P" -maxdepth 3 -name "*.yaml" | head -15
  echo "--- ROS2 markers (create_subscription) ---"
  grep -rl "create_subscription" "$P" --include=*.cpp | head -3
  echo "--- ROS1 markers (ros::Subscriber) ---"
  grep -rl "ros::Subscriber" "$P" --include=*.cpp | head -3
  echo "--- CustomMsg support ---"
  grep -rl "CustomMsg" "$P" --include=*.cpp --include=*.hpp | head -5
done
