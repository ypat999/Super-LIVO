#!/usr/bin/env bash
set -e
mkdir -p /home/ywj/slam_data/pl_ros2_ws/src
cd /home/ywj/slam_data/pl_ros2_ws
rm -rf src/point_lio
git clone --depth 1 -b grid_map_ros2 https://github.com/SMBU-PolarBear-Robotics-Team/point_lio.git src/point_lio 2>&1 | tail -1
R=src/point_lio
echo "=== layout ==="
ls "$R"
echo "=== config ==="
ls "$R/config" 2>/dev/null
echo "=== executable name ==="
grep -E "add_executable" "$R/CMakeLists.txt"
echo "=== topics published ==="
grep -nE "create_publisher" "$R/src/laserMapping.cpp" | head -15
echo "=== avia.yaml ==="
cat "$R/config/avia.yaml" 2>/dev/null | head -70
echo "=== lidar_type enum ==="
grep -nE "AVIA|MID360|Mid360|lidar_type" "$R/src/preprocess.h" | head
