#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
echo "=== daemon stop ==="
timeout 10 ros2 daemon stop 2>&1 | head -3
echo "=== pub test ==="
timeout 15 ros2 topic pub -1 /foobar std_msgs/msg/String "{data: hi}" >/tmp/pubtest.log 2>&1 &
sleep 4
echo "=== topic list in another process ==="
timeout 10 ros2 topic list --no-daemon 2>&1 | head -10
echo "=== pubtest.log ==="
cat /tmp/pubtest.log
echo "=== gdb ==="
command -v gdb || echo none
echo "=== shm count ==="
ls /dev/shm | wc -l
