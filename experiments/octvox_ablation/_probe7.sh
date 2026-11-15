#!/usr/bin/env bash
killall -q ros2 2>/dev/null
source /opt/ros/humble/setup.bash
unset ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS RMW_IMPLEMENTATION
echo "=== plain ros2 topic list ==="
timeout 15 ros2 topic list 2>&1 | tail -30
