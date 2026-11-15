#!/usr/bin/env bash
# 诊断：节点运行时，/left_camera/image 在总线上到底还有没有流量。
# 复刻 _replay_node_on_record.sh 的对齐方式（raw 先起播 0.39 s），中途多次量 hz。
set -o pipefail
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/fastlivo2_ws/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash

OUT=/tmp/probe_replay_monitor
rm -rf "$OUT"; mkdir -p "$OUT"
FL2BAG=/home/ywj/slam_data/official_vio_20261114/bag
RAWBAG=/home/ywj/slam_data/Retail_Street
ACFG=/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml
ABIN=/home/ywj/git/Super-LIVO/install/super_livo_visual_a/lib/super_livo_visual_a/visual_a_node

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

ros2 bag play "$RAWBAG" --rate 1.0 --disable-keyboard-controls < /dev/null > "$OUT/play_raw.log" 2>&1 &
P2=$!
sleep 0.39
ros2 bag play "$FL2BAG" --rate 1.0 --disable-keyboard-controls < /dev/null > "$OUT/play_fl2.log" 2>&1 &
P1=$!

for t in 5 20 40; do
  sleep $t
  echo "=== +${t}s image hz ===" >> "$OUT/hz.log"
  timeout 8 ros2 topic hz /left_camera/image >> "$OUT/hz.log" 2>&1
  echo "=== +${t}s body hz ===" >> "$OUT/hz.log"
  timeout 8 ros2 topic hz /cloud_registered_body >> "$OUT/hz.log" 2>&1
done

kill -INT "$P1" "$P2" 2>/dev/null; sleep 1
kill -INT "$VIS" 2>/dev/null
for _ in $(seq 1 40); do kill -0 "$VIS" 2>/dev/null || break; sleep 1; done
kill -9 "$VIS" "$P1" "$P2" 2>/dev/null

echo "===== 总线 hz ====="; cat "$OUT/hz.log"
echo "===== 节点统计 ====="; grep -a "彩色统计" "$OUT/vis.log" | tail -1
echo "===== 视觉帧日志条数 ====="; grep -ac "视觉帧" "$OUT/vis.log"
echo DONE_PROBE_REPLAY_MONITOR
