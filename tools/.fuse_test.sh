#!/usr/bin/env bash
# 临时实验：Retail_Street 方案A 体素颜色融合验证（默认融合参数，不加遮挡收紧覆盖）
set -o pipefail

SL_CFG=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/r3live.yaml
A_CFG=/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml
BAG=/home/ywj/slam_data/Retail_Street
OUT=/home/ywj/slam_data/pcd/fuse_test

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
source /tmp/sl_install/setup.bash

pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null
sleep 2
rm -rf "$OUT"
mkdir -p "$OUT"

ros2 run super_lio super_lio_node --ros-args --params-file "$SL_CFG" > /tmp/slio_fuse.log 2>&1 &
SL_PID=$!
ros2 run super_livo_visual_a visual_a_node --ros-args --params-file "$A_CFG" \
  -p pcd.save_dir:="$OUT" -p pcd.save_interval_sec:=0.0 \
  > /tmp/vis_fuse.log 2>&1 &
A_PID=$!
sleep 6
ros2 bag play "$BAG" --rate 1.0 > /tmp/bagplay_fuse.log 2>&1
sleep 3
pkill -INT -x visual_a_node; sleep 8
pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null

echo "=== 收尾 ==="
grep -a "体素融合" /tmp/vis_fuse.log | tail -2
ls -l "$OUT"
