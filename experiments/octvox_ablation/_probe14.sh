#!/usr/bin/env bash
pkill -9 -f 'ros2-daemon' 2>/dev/null; pkill -9 -f '_ros2_daemon' 2>/dev/null; sleep 1
source /opt/ros/humble/setup.bash
unset ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
echo "=== echo bg (no-daemon, no localhost-only) ==="
timeout 30 ros2 topic echo --no-daemon /foobar std_msgs/msg/String > /tmp/echo5.log 2>&1 &
sleep 6
echo "=== publish (no-daemon) ==="
timeout 12 ros2 topic pub --no-daemon -1 /foobar std_msgs/msg/String "{data: hi}" 2>&1 | head -4
sleep 3
echo "=== echo5.log ==="
head -12 /tmp/echo5.log
echo "=== list --no-daemon ==="
timeout 12 ros2 topic list --no-daemon 2>&1 | head -8
