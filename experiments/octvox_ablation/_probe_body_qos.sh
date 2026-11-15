#!/usr/bin/env bash
# 诊断：为什么同 run 回放时 [彩色统计] 调用=0（即 onBodyCloud 一次都没进来）
# 只回放 FL2 录制包（不含图像），观察 /cloud_registered_body 在不同 QoS 下的实际到达率。
set -o pipefail
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash
source /home/ywj/fastlivo2_ws/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash

OUT=/tmp/body_qos_probe
rm -rf "$OUT"; mkdir -p "$OUT"
FL2BAG=/home/ywj/slam_data/official_vio_20261114/bag
ACFG=/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml
ABIN=/home/ywj/git/Super-LIVO/install/super_livo_visual_a/lib/super_livo_visual_a/visual_a_node

# ---------- 实验 1：只放包，测两种 QoS 的到达率 ----------
echo "===== EXP1: rate under reliable vs best_effort ====="
timeout 15 ros2 bag play "$FL2BAG" --rate 1.0 --disable-keyboard-controls < /dev/null > "$OUT/play1.log" 2>&1 &
timeout 15 ros2 topic hz /cloud_registered_body > "$OUT/hz_reliable.log" 2>&1 &
timeout 15 ros2 topic hz /cloud_registered_body --qos-reliability best_effort > "$OUT/hz_be.log" 2>&1 &
timeout 15 ros2 topic hz /cloud_registered > "$OUT/hz_cr.log" 2>&1 &
wait
echo "--- /cloud_registered_body  reliable ---"; tail -2 "$OUT/hz_reliable.log"
echo "--- /cloud_registered_body  best_effort ---"; tail -2 "$OUT/hz_be.log"
echo "--- /cloud_registered      reliable ---"; tail -2 "$OUT/hz_cr.log"

# ---------- 实验 2：节点只收 body 云（无图像），看 调用 是否为 0 ----------
echo "===== EXP2: node + FL2 bag only (no images) ====="
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
timeout 20 ros2 bag play "$FL2BAG" --rate 1.0 --disable-keyboard-controls < /dev/null > "$OUT/play2.log" 2>&1
sleep 2
kill -INT "$VIS" 2>/dev/null
for _ in $(seq 1 30); do kill -0 "$VIS" 2>/dev/null || break; sleep 1; done
kill -9 "$VIS" 2>/dev/null
grep -a "彩色统计" "$OUT/vis.log" | tail -1
echo "--- 超时丢弃样本 ---"
grep -a "等待图像超时" "$OUT/vis.log" | tail -2
echo DONE_BODY_QOS_PROBE
