#!/usr/bin/env bash
killall -q ros2 2>/dev/null
source /opt/ros/humble/setup.bash
unset ROS_LOCALHOST_ONLY
unset FASTDDS_BUILTIN_TRANSPORTS
unset RMW_IMPLEMENTATION
echo "=== ifaces ==="
ip -4 addr | grep inet
echo "=== default FastDDS, no localhost only ==="
timeout 15 ros2 topic pub -1 /foobar std_msgs/msg/String "{data: hi}" >/tmp/pubtest3.log 2>&1 &
sleep 5
timeout 10 ros2 topic list 2>&1 | head -10
echo "=== pub log ==="
cat /tmp/pubtest3.log
