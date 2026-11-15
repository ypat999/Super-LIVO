#!/usr/bin/env bash
# 诊断：在与 _replay_node_on_record.sh 完全相同的环境下，单放原包时图像是否持续发布。
set -o pipefail
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/fastlivo2_ws/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash

OUT=/tmp/probe_img_hz_fullenv
rm -rf "$OUT"; mkdir -p "$OUT"
RAWBAG=/home/ywj/slam_data/Retail_Street

echo "RMW=$RMW_IMPLEMENTATION"
echo "CYCLONEDDS_URI=$CYCLONEDDS_URI"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"

ros2 bag play "$RAWBAG" --rate 1.0 --disable-keyboard-controls < /dev/null > "$OUT/play_raw.log" 2>&1 &
P=$!
sleep 2
timeout 12 ros2 topic hz /left_camera/image > "$OUT/hz_img.log" 2>&1
echo "=== hz image ==="; cat "$OUT/hz_img.log"
echo "=== play raw log ==="; cat "$OUT/play_raw.log"
kill -INT "$P" 2>/dev/null; sleep 1; kill -9 "$P" 2>/dev/null
echo DONE_PROBE_IMG_HZ
