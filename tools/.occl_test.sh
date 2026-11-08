#!/usr/bin/env bash
# 临时实验：收紧遮挡/深度边缘阈值，看盐噪声是否下降（Retail_Street，方案A）
set -o pipefail

SL_CFG=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/r3live.yaml
A_CFG=/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml
BAG=/home/ywj/slam_data/Retail_Street
OUT=/home/ywj/slam_data/pcd/occl_test

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
source /tmp/sl_install/setup.bash

pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null
sleep 2
rm -rf "$OUT"
mkdir -p "$OUT"

ros2 run super_lio super_lio_node --ros-args --params-file "$SL_CFG" > /tmp/slio_occl.log 2>&1 &
SL_PID=$!
ros2 run super_livo_visual_a visual_a_node --ros-args --params-file "$A_CFG" \
  -p pcd.save_dir:="$OUT" -p pcd.save_interval_sec:=0.0 \
  -p color.depth_tolerance:=0.02 -p color.edge_threshold:=0.1 \
  > /tmp/vis_occl.log 2>&1 &
A_PID=$!
sleep 6
ros2 bag play "$BAG" --rate 1.0 > /tmp/bagplay_occl.log 2>&1
sleep 3
pkill -INT -x visual_a_node; sleep 8
pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null

echo "=== 收尾 ==="
grep -a "彩色统计" /tmp/vis_occl.log | tail -1
ls -l "$OUT"
