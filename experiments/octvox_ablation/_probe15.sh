#!/usr/bin/env bash
pkill -9 -f 'ros2-daemon' 2>/dev/null; pkill -9 -f '_ros2_daemon' 2>/dev/null; sleep 1
source /opt/ros/humble/setup.bash
unset ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ywj/git/Super-LIVO/experiments/octvox_ablation/_fastdds_eth.xml
echo "=== echo bg ==="
timeout 30 ros2 topic echo --no-daemon /foobar std_msgs/msg/String > /tmp/echo6.log 2>&1 &
sleep 6
echo "=== publish ==="
timeout 12 ros2 topic pub -1 /foobar std_msgs/msg/String "{data: hi}" 2>&1 | head -4
sleep 3
echo "=== echo6.log ==="
head -12 /tmp/echo6.log
echo "=== list --no-daemon ==="
timeout 12 ros2 topic list --no-daemon 2>&1 | head -8
