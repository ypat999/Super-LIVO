#!/usr/bin/env bash
echo "=== daemon procs ==="
ps -eo pid,etime,cmd | grep -E '_ros2_daemon|ros2cli' | grep -v grep
echo "=== kill daemon ==="
pkill -f _ros2_daemon 2>/dev/null
sleep 1
ps -eo pid,cmd | grep _ros2_daemon | grep -v grep || echo "cleared"
echo "=== clean shm ==="
ls /dev/shm | grep -c fastrtps || true
rm -f /dev/shm/fastrtps_* 2>/dev/null
ls /dev/shm | wc -l

source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
echo "=== pub/list test after cleanup ==="
timeout 15 ros2 topic pub -1 /foobar std_msgs/msg/String "{data: hi}" >/tmp/pubtest4.log 2>&1 &
sleep 5
timeout 10 ros2 topic list --no-daemon 2>&1 | head -10
echo "=== pub log ==="
cat /tmp/pubtest4.log
