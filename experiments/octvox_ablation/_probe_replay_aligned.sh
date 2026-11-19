#!/usr/bin/env bash
# 验证：按 header 对齐（raw 包先起播 0.39 s）后，图像-点云配对是否恢复。
# 推导：wall(h) = W + (h + k - t0)，k = bagT - header；要求图像(h-0.1) 与 点云(h) 同刻
# => W_raw - W_fl2 = 0.1 + (t0_raw - k_raw) - (t0_fl2 - k_fl2) = -0.393 s
set -o pipefail
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash

OUT=/tmp/two_bag_aligned
rm -rf "$OUT"; mkdir -p "$OUT"
FL2BAG=/home/ywj/slam_data/official_vio_20261114/bag
RAWBAG=/home/ywj/slam_data/Retail_Street
ACFG=/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml
ABIN=/home/ywj/git/Super-LIVO/install/super_livo_visual_a/lib/super_livo_visual_a/visual_a_node

pkill -x visual_a_node >/dev/null 2>&1; sleep 1
"$ABIN" --ros-args --params-file "$ACFG" \
  -p super_livo_visual.image_topic:=/left_camera/image \
  -p super_livo_visual.odom_topic:=/aft_mapped_lidar_to_init \
  -p super_livo_visual.body_cloud_topic:=/cloud_registered_body \
  -p super_livo_visual.cloud_topic:=/cloud_registered \
  -p super_livo_visual.color.use_own_odom:=false \
  -p super_livo_visual.color.cam_odom_topic:=/vio/odom_lidar \
  -p super_livo_visual.pcd.save_dir:="$OUT" -p super_livo_visual.pcd.save_interval_sec:=0.0 \
  > "$OUT/vis.log" 2>&1 &
VIS=$!
sleep 4

ros2 bag play "$RAWBAG" --rate 1.0 --disable-keyboard-controls < /dev/null > "$OUT/play_raw.log" 2>&1 &
P2=$!
sleep 0.39
ros2 bag play "$FL2BAG" --rate 1.0 --disable-keyboard-controls < /dev/null > "$OUT/play_fl2.log" 2>&1 &
P1=$!

sleep 45
kill -INT "$P1" "$P2" 2>/dev/null; sleep 1
kill -INT "$VIS" 2>/dev/null
for _ in $(seq 1 40); do kill -0 "$VIS" 2>/dev/null || break; sleep 1; done
kill -9 "$VIS" "$P1" "$P2" 2>/dev/null

echo "===== 节点统计 ====="; grep -a "彩色统计" "$OUT/vis.log" | tail -1
echo "===== 坏例（超时） ====="; grep -a "等待图像超时" "$OUT/vis.log" | tail -2
echo "===== PCD ====="; ls -l "$OUT"/*.pcd 2>/dev/null || echo "无 PCD"
echo DONE_ALIGNED
