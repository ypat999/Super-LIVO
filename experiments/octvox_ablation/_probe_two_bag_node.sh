#!/usr/bin/env bash
# 复现「两包同时回放 + 节点」场景，并在运行中途抓订阅匹配状态，
# 用独立订阅者(hz, reliable)确认 body 云在总线上是否仍在发布。
set -o pipefail
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash

OUT=/tmp/two_bag_node
rm -rf "$OUT"; mkdir -p "$OUT"
FL2BAG=/home/ywj/slam_data/official_vio_20261114/bag
RAWBAG=/home/ywj/slam_data/Retail_Street
ACFG=/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml
ABIN=/home/ywj/git/Super-LIVO/install/super_livo_visual_a/lib/super_livo_visual_a/visual_a_node

pkill -x visual_a_node >/dev/null 2>&1; sleep 1
"$ABIN" --ros-args --params-file "$ACFG" \
  -p image_topic:=/left_camera/image \
  -p odom_topic:=/aft_mapped_lidar_to_init \
  -p body_cloud_topic:=/cloud_registered_body \
  -p cloud_topic:=/cloud_registered \
  -p color.use_own_odom:=false \
  -p color.cam_odom_topic:=/vio/odom_lidar \
  -p pcd.save_dir:="$OUT" -p pcd.save_interval_sec:=0.0 \
  > "$OUT/vis.log" 2>&1 &
VIS=$!
sleep 4

ros2 bag play "$FL2BAG" --rate 1.0 --disable-keyboard-controls < /dev/null > "$OUT/play_fl2.log" 2>&1 &
P1=$!
ros2 bag play "$RAWBAG" --delay 2.61 --rate 1.0 --disable-keyboard-controls < /dev/null > "$OUT/play_raw.log" 2>&1 &
P2=$!

sleep 12
{
  echo "=== nodes ==="; ros2 node list 2>&1
  echo "=== body topic info -v ==="; ros2 topic info -v /cloud_registered_body 2>&1
  echo "=== image topic info -v ==="; ros2 topic info -v /left_camera/image 2>&1
} > "$OUT/graph.txt" 2>&1
# 独立订阅者（reliable）确认 body 云在总线上是否真的在发
timeout 8 ros2 topic hz /cloud_registered_body > "$OUT/hz_body_ext.log" 2>&1
timeout 8 ros2 topic hz /left_camera/image    > "$OUT/hz_img_ext.log" 2>&1

sleep 40
kill -INT "$P1" "$P2" 2>/dev/null
sleep 1
kill -INT "$VIS" 2>/dev/null
for _ in $(seq 1 40); do kill -0 "$VIS" 2>/dev/null || break; sleep 1; done
kill -9 "$VIS" 2>/dev/null
kill -9 "$P1" "$P2" 2>/dev/null

echo "===== graph ====="; cat "$OUT/graph.txt"
echo "===== hz body(ext reliable) ====="; tail -2 "$OUT/hz_body_ext.log"
echo "===== hz image(ext reliable) ====="; tail -2 "$OUT/hz_img_ext.log"
echo "===== 视觉帧 条数 ====="; grep -ac "视觉帧" "$OUT/vis.log"
echo "===== 节点统计 ====="; grep -a "彩色统计" "$OUT/vis.log" | tail -1
echo DONE_TWO_BAG_NODE
