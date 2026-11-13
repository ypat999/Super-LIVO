#!/usr/bin/env bash
# 开源 Point-LIO ROS2 移植（SMBU-PolarBear-Robotics-Team/point_lio @ grid_map_ros2，
# 基于官方 hku-mars Point-LIO point-lio-with-grid-map 分支）完整跑 Retail_Street，
# 沿用方案A（紧凑 KLT colorizer）上色流程，输出彩色点云。
# 使用其自带 config/avia.yaml 默认参数，仅打开 body 系点云发布。
set -o pipefail
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
source /home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/install/setup.bash
source /home/ywj/slam_data/pl_ros2_ws/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash

VCFG=/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml
BAG=/home/ywj/slam_data/Retail_Street
OUT=/home/ywj/slam_data/colored_lio_20261110/pointlio_smbu
rm -rf "$OUT"; mkdir -p "$OUT"

pkill -x pointlio_mappin 2>/dev/null; pkill -x visual_a_node 2>/dev/null; sleep 2

ros2 run point_lio pointlio_mapping --ros-args \
  --params-file /home/ywj/slam_data/pl_ros2_ws/src/point_lio/config/avia.yaml \
  -p publish.scan_bodyframe_pub_en:=True \
  > "$OUT/lio.log" 2>&1 &
sleep 5
ros2 run super_livo_visual_a visual_a_node --ros-args \
  --params-file "$VCFG" \
  -p odom_topic:="/aft_mapped_to_init" \
  -p body_cloud_topic:="/cloud_registered_body" \
  -p cloud_topic:="/cloud_registered" \
  -p pcd.save_dir:="$OUT" \
  -p pcd.save_interval_sec:=0.0 \
  > "$OUT/vis.log" 2>&1 &
sleep 3
ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null > "$OUT/play.log" 2>&1
sleep 15                                   # 若 LIO 落后于 bag，留追平余量
pkill -INT -x visual_a_node; sleep 60      # 退出时落盘彩色地图
pkill -x pointlio_mappin 2>/dev/null; pkill -x visual_a_node 2>/dev/null; sleep 3
grep -a "彩色地图已保存" "$OUT/vis.log" | tail -1
grep -a "彩色统计" "$OUT/vis.log" | tail -1
tail -5 "$OUT/lio.log"
ls -l "$OUT"/*.pcd 2>/dev/null || echo "!!! pointlio_smbu 无 PCD 输出"
echo DONE_COLORED_POINTLIO_SMBU
