#!/usr/bin/env bash
pkill -f 'ros2cli.daemon' 2>/dev/null; pkill -f 'ros2-daemon' 2>/dev/null; sleep 1
source /opt/ros/humble/setup.bash
unset ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
echo "=== echo bg (default fastdds, all ifaces) ==="
timeout 25 ros2 topic echo /foobar std_msgs/msg/String > /tmp/echo4.log 2>&1 &
sleep 5
echo "=== publish ==="
timeout 12 ros2 topic pub -1 /foobar std_msgs/msg/String "{data: hi}" 2>&1 | head -4
sleep 2
echo "=== echo4.log ==="
head -12 /tmp/echo4.log
echo "=== list --no-daemon ==="
timeout 12 ros2 topic list --no-daemon 2>&1 | head -8
