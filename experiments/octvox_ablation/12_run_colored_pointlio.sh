#!/usr/bin/env bash
# Point-LIO（打补丁版，/home/ywj/slam_data/pl_ws）完整跑 Retail_Street，
# 沿用方案A（紧凑 KLT colorizer）上色流程，输出彩色点云。
# 补丁内容：laserMapping.cpp 三处 imu_deque.front()/pop_front() 循环加 !empty() 保护
#（原版在 IMU 初始化完成后因空 deque 取 front 陷入死循环，零输出）。
# 另需 publish.scan_bodyframe_pub_en:=true 才有 /cloud_registered_body（avia.yaml 默认 false）。
set -o pipefail
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
source /home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/install/setup.bash
source /home/ywj/slam_data/pl_ws/install/setup.bash   # 补丁版 point_lio（必须最后 source 以覆盖）
source /home/ywj/git/Super-LIVO/install/setup.bash

VCFG=/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml
BAG=/home/ywj/slam_data/Retail_Street
OUT=/home/ywj/slam_data/colored_lio_20261110/pointlio
rm -rf "$OUT"; mkdir -p "$OUT"

pkill -x pointlio_mappin 2>/dev/null; pkill -x visual_a_node 2>/dev/null; sleep 2

ros2 run point_lio pointlio_mapping --ros-args \
  --params-file /home/ywj/slam_data/pl_ws/src/point_lio/config/avia.yaml \
  -p publish.scan_bodyframe_pub_en:=true \
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
grep -a "\[patch\]" "$OUT/lio.log" | head -3
ls -l "$OUT"/*.pcd 2>/dev/null || echo "!!! pointlio 无 PCD 输出"
echo DONE_COLORED_POINTLIO
