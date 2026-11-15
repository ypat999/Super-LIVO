#!/usr/bin/env bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash
ros2 topic info /cloud_registered -v 2>&1 | head -40
echo ====================
ros2 topic info /cloud_registered_body -v 2>&1 | head -40
