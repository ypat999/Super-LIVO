#!/usr/bin/env bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash
echo "=== topic hz (3s) ==="
for t in /cloud_registered_body /aft_mapped_lidar_to_init /vio/odom_lidar /left_camera/image; do
  echo "-- $t"
  timeout 4 ros2 topic hz "$t" --window 20 2>&1 | tail -2
done
echo "=== info -v body ==="
ros2 topic info -v /cloud_registered_body 2>&1 | head -20
