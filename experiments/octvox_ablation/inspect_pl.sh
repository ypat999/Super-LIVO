#!/usr/bin/env bash
R=/home/ywj/slam_data/pl_official_ws/src/point_lio_unilidar
echo "=== package.xml name/deps ==="
grep -E "<name>|depend" "$R/package.xml"
echo "=== CMake executable ==="
grep -E "add_executable|ament_" "$R/CMakeLists.txt" | head
echo "=== pub/sub topics in laserMapping.cpp ==="
grep -nE "create_publisher|create_subscription" "$R/src/laserMapping.cpp" | head -20
echo "=== bodyframe related params ==="
grep -nE "body|Body" "$R/src/laserMapping.cpp" | head -20
grep -nE "body" "$R/config/avia.yaml"
echo "=== avia.yaml ==="
cat "$R/config/avia.yaml"
echo "=== README run section ==="
grep -nE "ros2 run|ros2 launch|colcon|source" "$R/README.md" | head -20
