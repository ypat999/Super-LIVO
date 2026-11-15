#!/usr/bin/env bash
kill -9 1015954 2>&1
sleep 2
ps -eo pid,cmd | grep -E 'ros2-daemon|ros2cli' | grep -v grep || echo "daemon gone"
ls -la ~/.ros/ 2>/dev/null | head
rm -f /dev/shm/fastrtps_* 2>/dev/null

source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
echo "=== echo bg ==="
timeout 25 ros2 topic echo /foobar std_msgs/msg/String > /tmp/echo3.log 2>&1 &
sleep 5
echo "=== publish ==="
timeout 10 ros2 topic pub -1 /foobar std_msgs/msg/String "{data: hi}" 2>&1 | head -3
sleep 2
echo "=== echo3.log ==="
head -12 /tmp/echo3.log
echo "=== list --no-daemon ==="
timeout 10 ros2 topic list --no-daemon 2>&1 | head -8
