#!/usr/bin/env bash
# 方案 A（Super-LIO 主干 + visual_a_node 上色）在 move_ros2 上的整段彩色重建（两进程模式）。
# 图像 header 是主机时钟、LIO 是设备时钟；用 20_move_clock_shift_relay.py 把 LIO 侧
# odom/点云抬到主机时钟域（只平移这两个话题，绝不平移 /tf，否则外参推相机中心全错）。
# 用法: bash 20a_run_move_a.sh [tag]        默认 tag=move_hf_a，倍速 RATE=0.6
# 实测（2026-11-17 holefix 外参，RATE=0.6）：76.4s 数据 → 739 帧、path 9.581 m、闭环 0.027 m、
# 彩色图 3.38 M 点（721/741 帧着色，无 >0.3 m 跳变）。
set -o pipefail
REPO="${REPO:-$HOME/git/Super-LIVO}"
EXP=$REPO/experiments/octvox_ablation
TAG="${1:-move_hf_a}"
RESULTS="${RESULTS:-/mnt/dataset/slam_data/calib/results}"
OUT=$RESULTS/$TAG
BAG="${BAG:-/mnt/dataset/slam_data/calib/move_ros2}"
SLIO_CFG=$REPO/common/super_livo_common/config/lio_move.yaml
VIS_CFG=$EXP/20_move_a_node.yaml

export LD_LIBRARY_PATH="$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -vE 'trae|^\s*$' | paste -sd:)"
source /opt/ros/humble/setup.bash
source "$HOME/git/xtd2_ws/install/setup.bash" >/dev/null 2>&1 || true
source $REPO/third_party/Super-LIO/install/setup.bash
source $REPO/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

mkdir -p "$OUT"; rm -rf "$OUT"/*
pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null
pkill -f 'stamp_shift_relay|clock_shift_relay' 2>/dev/null; pkill -f 'odom_rec[o]der' 2>/dev/null
sleep 2

nohup python3 "$EXP/20_move_clock_shift_relay.py" > "$OUT/relay.log" 2>&1 &
nohup python3 $REPO/third_party/Super-LIO/exp/odom_recorder.py \
  --out "$OUT/odom.csv" --topic /lio/odom_shift > "$OUT/rec.log" 2>&1 &
ros2 run super_lio super_lio_node --ros-args \
  --params-file "$SLIO_CFG" \
  -p lio.map.save_map:=false -p lio.map.save_interval:=-1 \
  > "$OUT/slio.log" 2>&1 &
ros2 run super_livo_visual_a visual_a_node --ros-args \
  --params-file "$VIS_CFG" \
  -p pcd.save_dir:="$OUT" -p pcd.save_interval_sec:=0.0 \
  > "$OUT/vis.log" 2>&1 &

for i in $(seq 1 60); do grep -aq "Map init done" "$OUT/slio.log" && break; sleep 1; done
sleep 2
echo "=== [A] play bag (rate=${RATE:-0.6}) ==="
ros2 bag play "$BAG" --rate "${RATE:-0.6}" --disable-keyboard-controls < /dev/null > "$OUT/play.log" 2>&1
echo "=== [A] bag finished, wait visual catch-up (max 150s) ==="
prev=0
for i in $(seq 1 30); do
  sleep 5
  cur=$(wc -l < "$OUT/vis.log")
  [ "$cur" = "$prev" ] && break
  prev=$cur
done
pkill -INT -x visual_a_node 2>/dev/null; sleep 50      # SIGINT 后 A 会落盘彩色地图，必须等
pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null
pkill -f 'stamp_shift_relay|clock_shift_relay' 2>/dev/null; pkill -f 'odom_rec[o]der' 2>/dev/null; sleep 2
grep -a "彩色地图已保存\|彩色统计" "$OUT/vis.log" | tail -2
ls -la "$OUT"
