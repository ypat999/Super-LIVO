#!/usr/bin/env bash
pkill -f _ros2_daemon 2>/dev/null; sleep 1
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
echo "=== start daemon ==="
timeout 20 ros2 daemon start 2>&1 | head -3
echo "=== echo in bg ==="
timeout 20 ros2 topic echo /foobar std_msgs/msg/String > /tmp/echo.log 2>&1 &
sleep 4
echo "=== publish ==="
timeout 10 ros2 topic pub -1 /foobar std_msgs/msg/String "{data: hi}" 2>&1 | head -5
sleep 2
echo "=== echo.log ==="
cat /tmp/echo.log
echo "=== topic list (daemon) ==="
timeout 15 ros2 topic list 2>&1 | head -10
echo "=== node list ==="
timeout 15 ros2 node list 2>&1 | head -5
