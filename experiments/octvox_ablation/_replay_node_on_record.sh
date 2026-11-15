#!/usr/bin/env bash
# 同一 run 内比色：不启动 FL2，直接回放"官方 VIO 录制包"（位姿+body云+官方染色云）
# 与 Retail_Street 原包（图像），让节点 A 在同一批数据上着色。
# 这样节点产出的几何与离线 ladder R5 完全同源（最近邻距离应为 0），2cm/最近邻比色才有效。
set -o pipefail
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/fastlivo2_ws/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash

# PLAN=A|B：用同一份录制流分别跑 A / B 节点，做"输入完全相同"的等价对比。
PLAN=${PLAN:-A}
OUT=${OUT:-/home/ywj/slam_data/replay_node_onrecord_20261115}
FL2BAG=/home/ywj/slam_data/official_vio_20261114/bag
RAWBAG=/home/ywj/slam_data/Retail_Street
ACFG=/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml
BCFG=/home/ywj/git/Super-LIVO/solution_b_tight_coupled/super_livo_visual_b/config/visual_b_params.yaml
ABIN=/home/ywj/git/Super-LIVO/install/super_livo_visual_a/lib/super_livo_visual_a/visual_a_node
BBIN=/home/ywj/git/Super-LIVO/install/super_livo_visual_b/lib/super_livo_visual_b/visual_b_node
if [ "$PLAN" = "B" ]; then CFG=$BCFG; BIN=$BBIN; else CFG=$ACFG; BIN=$ABIN; fi
mkdir -p "$OUT"
rm -f "$OUT"/*.pcd

pkill -x visual_a_node >/dev/null 2>&1
pkill -x visual_b_node >/dev/null 2>&1
sleep 1

"$BIN" --ros-args --params-file "$CFG" \
  -p image_topic:=/left_camera/image \
  -p odom_topic:=/aft_mapped_lidar_to_init \
  -p body_cloud_topic:=/cloud_registered_body \
  -p cloud_topic:=/cloud_registered \
  -p color.use_own_odom:=false \
  -p color.cam_odom_topic:=/vio/odom_lidar \
  -p color.patch_radius:=${PATCH_R:-0} \
  -p color.patch_color_tol:=${PATCH_TOL:-20.0} \
  -p color.check_depth_edges:=${CHECK_EDGES:-true} \
  -p color.depth_tolerance:=${DEPTH_TOL:-0.1} \
  -p color.min_quality:=${MINQ:-0.2} \
  -p pcd.save_dir:="$OUT" -p pcd.save_interval_sec:=0.0 \
  -p pcd.median_voxel_size:=${MEDIAN_VOXEL:-0.01} \
  > "$OUT/vis.log" 2>&1 &
VIS=$!
sleep 4

# 两个包的录制时间轴起点不同，必须按 header 对齐，否则图像与点云相差数秒，
# 超出 color.sync_max_offset(0.05 s) 配对窗，节点会一帧都染不上。
# 推导：ros2 bag play 按 bag timestamp 调度，wall(h) = W + (h + k - t0)，k = bagT - header；
# 要求图像(header = h-0.1) 与点云(header = h) 同刻到达
#   => W_raw - W_fl2 = 0.1 + (t0_raw - k_raw) - (t0_fl2 - k_fl2) = -0.393 s
# 即原包（图像）先起播 0.393 s，FL2 包随后。
ros2 bag play "$RAWBAG" --rate 1.0 --disable-keyboard-controls < /dev/null > "$OUT/play_raw.log" 2>&1 &
P2=$!
sleep 0.39
ros2 bag play "$FL2BAG" --rate 1.0 --disable-keyboard-controls < /dev/null > "$OUT/play_fl2.log" 2>&1 &
P1=$!
wait $P1
wait $P2
sleep 3

kill -INT "$VIS" 2>/dev/null
for _ in $(seq 1 60); do
  kill -0 "$VIS" 2>/dev/null || break
  sleep 2
done
kill -9 "$VIS" 2>/dev/null
sleep 1
grep -a "彩色地图已保存" "$OUT/vis.log" | tail -1
grep -a "彩色统计" "$OUT/vis.log" | tail -1
ls -l "$OUT"/*.pcd 2>/dev/null || echo "!!! 无 PCD 输出"
echo DONE_REPLAY_NODE_ON_RECORD
