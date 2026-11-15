#!/usr/bin/env bash
echo "=== all ros2-ish procs ==="
ps -eo pid,etime,cmd | grep -E 'ros2cli|ros2-daemon|_ros2_daemon' | grep -v grep
echo "=== kill them ==="
pkill -f 'ros2cli.daemon' 2>/dev/null
pkill -f 'ros2-daemon' 2>/dev/null
sleep 2
ps -eo pid,cmd | grep -E 'ros2cli|ros2-daemon' | grep -v grep || echo "all cleared"
echo "=== shm ==="
rm -f /dev/shm/fastrtps_* 2>/dev/null
ls /dev/shm | wc -l

source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
echo "=== --no-daemon list, no other node ==="
timeout 10 ros2 topic list --no-daemon 2>&1 | head -5
echo "=== echo bg ==="
timeout 20 ros2 topic echo /foobar std_msgs/msg/String > /tmp/echo2.log 2>&1 &
sleep 4
echo "=== publish ==="
timeout 10 ros2 topic pub -1 /foobar std_msgs/msg/String "{data: hi}" 2>&1 | head -3
sleep 2
echo "=== echo2.log ==="
head -10 /tmp/echo2.log
echo "=== list --no-daemon during ==="
timeout 10 ros2 topic list --no-daemon 2>&1 | head -8
