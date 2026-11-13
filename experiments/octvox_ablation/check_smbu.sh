#!/usr/bin/env bash
echo "=== SMBU point_lio: package.xml + livox cbk ==="
curl -s https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/point_lio/main/package.xml | grep -E "<name>|format|depend" | head -15
echo "---"
curl -s "https://api.github.com/repos/SMBU-PolarBear-Robotics-Team/point_lio/branches" | grep '"name"'
echo "=== search code for CustomMsg ==="
curl -s "https://api.github.com/search/code?q=repo:SMBU-PolarBear-Robotics-Team/point_lio+livox_ros_driver2" -H "Accept: application/vnd.github+json" | grep -E '"path"|"name"' | head -10
echo "=== dfloreaa preprocess: Livox PointCloud2 handling ==="
grep -nE "case Livox|Livox|custom" /home/ywj/slam_data/pl_official_ws/src/point_lio/src/preprocess.h | head
grep -nE "process_pcl_msg|process_live_msg|lidar_type" /home/ywj/slam_data/pl_official_ws/src/point_lio/src/preprocess.cpp | head -20
