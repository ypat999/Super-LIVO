#!/usr/bin/env bash
# FL2 位姿 × 方案A/B 着色：整段 Retail_Street，官方 FAST-LIVO2（视觉修正后位姿）
# 提供位姿与体系点云，方案 A / B 的着色节点只负责贴图。
# 与 10_run_colored_bc.sh（Super-LIO 位姿）、11~13_run_colored_*（FAST-LIO2 / Point-LIO 位姿）
# 构成「同一点云+图像、只换位姿来源」的对照。
#
# 依赖 FL2 fork 新增的两个话题（见 LIVMapper.cpp handleLIO）：
#   /cloud_registered_body     当前帧 LiDAR 系去畸变点云（供逐点运动补偿）
#   /aft_mapped_lidar_to_init  同刻 LiDAR 位姿（FL2 原生 /aft_mapped_to_init 是 IMU 位姿）
#
#   bash experiments/octvox_ablation/14_run_colored_fl2_pose.sh
# 产物：$OUT/fl2_{A,B}/{super_livo_visual_{a,b}_colored.pcd, fl2.log, vis.log, play.log}
set -o pipefail
# 本机 WSL 的 lo 不支持 multicast → FastDDS 发现失效，统一改用 CycloneDDS + 物理网卡。
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/fastlivo2_ws/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash

BAG=/home/ywj/slam_data/Retail_Street
# OUT / OWN 可由环境变量覆盖：OWN=true 时上色相机位姿改用 A/B 各自的视觉修正轨迹
OUT=${OUT:-/home/ywj/slam_data/colored_fl2_20261114}
OWN=${OWN:-false}
BIN=/home/ywj/fastlivo2_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping
ACFG=/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml
BCFG=/home/ywj/git/Super-LIVO/solution_b_tight_coupled/super_livo_visual_b/config/visual_b_params.yaml
# 直接起可执行文件：`ros2 run` 只把 SIGINT 转发成 SIGTERM，节点来不及在析构里落盘
ABIN=/home/ywj/git/Super-LIVO/install/super_livo_visual_a/lib/super_livo_visual_a/visual_a_node
BBIN=/home/ywj/git/Super-LIVO/install/super_livo_visual_b/lib/super_livo_visual_b/visual_b_node
mkdir -p "$OUT"

CFG=$OUT/avia_wrapped_pair.yaml
python3 /home/ywj/git/Super-LIVO/experiments/octvox_ablation/07a_wrap_official_cfg.py \
    "$BAG/avia.yaml" "$CFG" "$BAG/camera_pinhole.yaml"

run_one() {
  plan=$1
  dir=$OUT/fl2_$plan
  rm -rf "$dir"; mkdir -p "$dir"
  echo "=================== FL2-POSE COLORED RUN $plan ==================="
  pkill -x fastlivo_mapping >/dev/null 2>&1
  pkill -x visual_a_node >/dev/null 2>&1
  pkill -x visual_b_node >/dev/null 2>&1
  sleep 2

  $BIN --ros-args --params-file "$CFG" \
      -p common.img_en:=1 \
      > "$dir/fl2.log" 2>&1 &
  FL2=$!
  sleep 10
  if ! kill -0 "$FL2" 2>/dev/null; then
    echo "[fl2_$plan] !! FL2 启动失败"; tail -25 "$dir/fl2.log"; return
  fi

  # 官方口径：点云世界变换用 LIO 位姿（odom_topic=/aft_mapped_lidar_to_init），
  # 相机位姿用 VIO 光度修正后的位姿（/vio/odom_lidar）。OWN=true 时相机位姿改用
  # A/B 各自的视觉修正轨迹，不再另给相机位姿源。
  EXTRA=()
  if [ "$OWN" != "true" ]; then
    EXTRA+=(-p super_livo_visual.color.cam_odom_topic:=/vio/odom_lidar)
  fi

  if [ "$plan" = "A" ]; then
    "$ABIN" --ros-args --params-file "$ACFG" \
      -p super_livo_visual.odom_topic:=/aft_mapped_lidar_to_init \
      -p super_livo_visual.body_cloud_topic:=/cloud_registered_body \
      -p super_livo_visual.cloud_topic:=/cloud_registered \
      -p super_livo_visual.color.use_own_odom:="$OWN" \
      "${EXTRA[@]}" \
      -p super_livo_visual.pcd.save_dir:="$dir" -p super_livo_visual.pcd.save_interval_sec:=0.0 \
      > "$dir/vis.log" 2>&1 &
  else
    "$BBIN" --ros-args --params-file "$BCFG" \
      -p super_livo_visual.odom_topic:=/aft_mapped_lidar_to_init \
      -p super_livo_visual.body_cloud_topic:=/cloud_registered_body \
      -p super_livo_visual.cloud_topic:=/cloud_registered \
      -p super_livo_visual.color.use_own_odom:="$OWN" \
      "${EXTRA[@]}" \
      -p super_livo_visual.pcd.save_dir:="$dir" -p super_livo_visual.pcd.save_interval_sec:=0.0 \
      > "$dir/vis.log" 2>&1 &
  fi
  VIS=$!
  sleep 5

  ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null \
      > "$dir/play.log" 2>&1
  sleep 3

  kill -INT "$VIS" 2>/dev/null          # 节点析构里 drain + 落盘彩色地图
  for _ in $(seq 1 60); do
    kill -0 "$VIS" 2>/dev/null || break
    sleep 2
  done
  kill -9 "$VIS" 2>/dev/null
  kill -INT "$FL2" 2>/dev/null; sleep 5; kill -9 "$FL2" 2>/dev/null
  sleep 2
  grep -a "彩色地图已保存" "$dir/vis.log" | tail -1
  ls -l "$dir"/*.pcd 2>/dev/null || echo "!!! fl2_$plan 无 PCD 输出"
}

run_one A
run_one B
echo DONE_FL2_POSE_COLORED
