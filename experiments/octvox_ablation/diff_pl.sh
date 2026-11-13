#!/usr/bin/env bash
DOG=/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/point_lio_ros2
NEW=/home/ywj/slam_data/pl_official_ws/src/point_lio
for f in laserMapping.cpp parameters.cpp parameters.h preprocess.cpp preprocess.h; do
  echo "===== $f: $(diff "$DOG/src/$f" "$NEW/src/$f" | grep -cE '^[<>]') changed lines ====="
done
echo "===== laserMapping.cpp diff (first 120 lines) ====="
diff "$DOG/src/laserMapping.cpp" "$NEW/src/laserMapping.cpp" | head -120
echo "===== config diff avia.yaml ====="
diff "$DOG/config/avia.yaml" "$NEW/config/avia.yaml" | head -60
