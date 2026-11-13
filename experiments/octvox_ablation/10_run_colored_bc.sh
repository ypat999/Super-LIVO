#!/usr/bin/env bash
# 整段彩色点云肉眼检查：B(rep_mode=2) / C(rep_mode=1) 各完整跑一遍 Retail_Street，
# Super-LIO 与方案A着色节点（紧凑 KLT colorizer，全部默认参数）同时在线。
# 着色用 lio/odom（color_use_own_odom 默认 false），因此两组差异直接体现在
# 位姿质量 → 彩色地图的层裂/椒盐上。
# 产物：$OUT/rep_{B,C}/super_livo_visual_a_colored.pcd
set -o pipefail
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash

CFG=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/r3live.yaml
VCFG=/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml
BAG=/home/ywj/slam_data/Retail_Street
OUT=/home/ywj/slam_data/colored_bc_20261110
mkdir -p "$OUT"

for pair in B:2 C:1; do
  name="${pair%%:*}"; mode="${pair##*:}"
  dir="$OUT/rep_$name"
  rm -rf "$dir"; mkdir -p "$dir"
  echo "=================== COLORED RUN $name (rep_mode=$mode) ==================="
  pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null; sleep 2
  ros2 run super_lio super_lio_node --ros-args \
    --params-file "$CFG" -p lio.hash_map.rep_mode:="$mode" \
    > "$dir/slio.log" 2>&1 &
  ros2 run super_livo_visual_a visual_a_node --ros-args \
    --params-file "$VCFG" -p pcd.save_dir:="$dir" -p pcd.save_interval_sec:=0.0 \
    > "$dir/vis.log" 2>&1 &
  for i in $(seq 1 60); do grep -aq "Map init done" "$dir/slio.log" && break; sleep 1; done
  sleep 2
  ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null \
    > "$dir/play.log" 2>&1
  sleep 3
  pkill -INT -x visual_a_node; sleep 50          # 退出时落盘彩色地图
  pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null; sleep 3
  grep -a "彩色地图已保存" "$dir/vis.log" | tail -1
  ls -l "$dir"/*.pcd 2>/dev/null || echo "!!! $name 无 PCD 输出"
done
echo DONE_COLORED_BC
