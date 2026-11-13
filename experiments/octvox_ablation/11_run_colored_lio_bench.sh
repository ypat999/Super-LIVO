#!/usr/bin/env bash
# 用 FAST-LIO2 与 Point-LIO（dog_slam 工作区）各完整跑一遍 Retail_Street，
# 沿用方案A（紧凑 KLT colorizer）上色流程，输出彩色点云供肉眼对比。
# 着色输入映射：
#   odom_topic        -> 各自的后验位姿（IMU 位姿，Livox 集成 IMU-LiDAR 杆臂 ~4 cm，
#                        与 Super-LIO 的 lio/odom（LiDAR 位姿）语义略有差异，两组一致承担）
#   body_cloud_topic  -> /cloud_registered_body（去畸变体系云）
#   cloud_topic       -> /cloud_registered（世界系，深度关联用）
set -o pipefail
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
source /home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash

WS=/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2
VCFG=/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml
BAG=/home/ywj/slam_data/Retail_Street
OUT=/home/ywj/slam_data/colored_lio_20261110
mkdir -p "$OUT"

for sys in fastlio2 pointlio; do
  dir="$OUT/$sys"
  rm -rf "$dir"; mkdir -p "$dir"
  echo "=================== COLORED RUN $sys ==================="
  pkill -x fastlio_mapping 2>/dev/null; pkill -x pointlio_mapping 2>/dev/null
  pkill -x visual_a_node 2>/dev/null; sleep 2

  if [ "$sys" = "fastlio2" ]; then
    LCFG=$WS/src/FAST_LIO_ROS2_edit/config/avia.yaml
    ros2 run fast_lio fastlio_mapping --ros-args --params-file "$LCFG" \
      > "$dir/lio.log" 2>&1 &
    odom=/Odometry
  else
    LCFG=$WS/src/point_lio_ros2/config/avia.yaml
    ros2 run point_lio pointlio_mapping --ros-args --params-file "$LCFG" \
      > "$dir/lio.log" 2>&1 &
    odom=/aft_mapped_to_init
  fi
  sleep 5
  ros2 run super_livo_visual_a visual_a_node --ros-args \
    --params-file "$VCFG" \
    -p odom_topic:="$odom" \
    -p body_cloud_topic:="/cloud_registered_body" \
    -p cloud_topic:="/cloud_registered" \
    -p pcd.save_dir:="$dir" \
    -p pcd.save_interval_sec:=0.0 \
    > "$dir/vis.log" 2>&1 &
  sleep 3
  ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null \
    > "$dir/play.log" 2>&1
  sleep 3
  pkill -INT -x visual_a_node; sleep 50          # 退出时落盘彩色地图
  pkill -x fastlio_mapping 2>/dev/null; pkill -x pointlio_mapping 2>/dev/null
  pkill -x visual_a_node 2>/dev/null; sleep 3
  grep -a "彩色地图已保存" "$dir/vis.log" | tail -1
  ls -l "$dir"/*.pcd 2>/dev/null || echo "!!! $sys 无 PCD 输出"
done
echo DONE_COLORED_LIO
