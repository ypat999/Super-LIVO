#!/usr/bin/env bash
# 快速自检：官方节点是否真的在发布 /cloud_registered_body、/vio/odom_lidar
set -o pipefail
CFG=/home/ywj/slam_data/fastlivo2_ref_20261110/avia_wrapped.yaml
OUT=/tmp/official_vio_probe
rm -rf "$OUT"; mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/fastlivo2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

pkill -f fastlivo_mapping >/dev/null 2>&1; sleep 2
stdbuf -oL -eL ros2 run fast_livo fastlivo_mapping --ros-args --params-file "$CFG" > "$OUT/node.log" 2>&1 &
NODEPID=$!
sleep 10
echo "=== node alive ==="
ps -o pid,stat,etime,cmd -p $(pgrep -f fastlivo_mapping | head -1) 2>&1
echo "=== node.log head ==="
head -20 "$OUT/node.log"
echo "=== topic list ==="
ros2 topic list --no-daemon 2>&1 | head -40
echo "=== topic info ==="
ros2 topic info /cloud_registered_body --no-daemon -v 2>&1 | head -20
ros2 topic info /vio/odom_lidar --no-daemon -v 2>&1 | head -20
ros2 topic info /aft_mapped_lidar_to_init --no-daemon -v 2>&1 | head -20

ros2 bag record -o "$OUT/bag" /cloud_registered_body /vio/odom_lidar \
    /aft_mapped_lidar_to_init /cloud_registered > "$OUT/rec.log" 2>&1 &
recpid=$!
sleep 3
timeout 30 ros2 bag play /home/ywj/slam_data/Retail_Street --rate 1.0 \
    --disable-keyboard-controls > "$OUT/play.log" 2>&1
sleep 2
echo "=== node.log during run (tail) ==="
tail -20 "$OUT/node.log"
echo "=== hz ==="
timeout 6 ros2 topic hz /cloud_registered_body 2>&1 | tail -4
kill -INT "$recpid" 2>/dev/null; sleep 5
pkill -f fastlivo_mapping >/dev/null 2>&1; sleep 2
echo "=== bag info ==="
ros2 bag info "$OUT/bag" 2>&1 | sed -n '1,25p'
echo "=== node.log full tail ==="
tail -30 "$OUT/node.log"
