#!/usr/bin/env bash
# 用 CycloneDDS + 纯单播（127.0.0.1 peers）绕开多播发现问题
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>10</MaxAutoParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
echo "=== echo bg ==="
timeout 25 ros2 topic echo /foobar std_msgs/msg/String > /tmp/cecho.log 2>&1 &
sleep 5
echo "=== publish ==="
timeout 12 ros2 topic pub -1 /foobar std_msgs/msg/String "{data: hi}" 2>&1 | head -4
sleep 2
echo "=== cecho.log ==="
head -12 /tmp/cecho.log
echo "=== list ==="
timeout 12 ros2 topic list --no-daemon 2>&1 | head -8
