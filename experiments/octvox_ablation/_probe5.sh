#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "=== route multicast ==="
ip route show | grep -E '224|default' | head
echo "=== pub test (cyclone) ==="
timeout 15 ros2 topic pub -1 /foobar std_msgs/msg/String "{data: hi}" >/tmp/pubtest2.log 2>&1 &
sleep 4
timeout 10 ros2 topic list 2>&1 | head -10
echo "=== pubtest2.log ==="
cat /tmp/pubtest2.log
