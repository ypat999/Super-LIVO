#!/usr/bin/env bash
set -e
cd /home/ywj/slam_data/pl_official_ws
git clone --depth 1 https://github.com/dfloreaa/point_lio_ros2.git src/point_lio 2>&1 | tail -1
echo "=== pkg layout ==="
ls src/point_lio
grep -E "<name>" src/point_lio/package.xml
echo "=== find dog_slam pointlio dir ==="
ls /home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src
DOG=$(ls -d /home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/*point*lio* 2>/dev/null | head -1)
echo "DOG=$DOG"
echo "=== diff dog_slam vs dfloreaa original (names only) ==="
diff -rq "$DOG/src" src/point_lio/src 2>/dev/null | head -30 || true
echo "=== diff config ==="
ls "$DOG/config" 2>/dev/null; ls src/point_lio/config
echo "=== dfloreaa avia.yaml key params ==="
grep -nE "lidar_type|scan_line|extrinsic_T|extrinsic_R|imu_time_inte|acc_norm|satu_|timestamp_unit|blind|scan_bodyframe" src/point_lio/config/avia.yaml 2>/dev/null || true
