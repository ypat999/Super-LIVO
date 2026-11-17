#!/usr/bin/env bash
# 方案 B（Super-LIO 主干 + visual_b_node 紧耦合）在 move_ros2 上的整段彩色重建（两进程模式）。
# 与 20a 同一 LIO 主干、同一 relay 时间域、同一外参，只换视觉支路。
# 用法: bash 20b_run_move_b.sh [tag]        默认 tag=move_hf_b，倍速 RATE=0.05
#
# ⚠️ B 的吞吐是瓶颈：单帧约 2.8 s CPU，而 0.05 倍速下雷达帧到达间隔 2.1 s，节点追不上 →
#    丢帧发生在 DDS 订阅侧（日志里 队列溢出=0、等待图像超时很少），不是配对阈值调错。
#    RATE>0.12 会整段跑不完。实测（2026-11-17，RATE=0.05）：430/739 帧着色（58%），
#    时间覆盖完整（t_cloud 806.99→883.09），彩色图 2.30 M 点，轨迹仍全帧率
#    （739 帧、path 9.561 m、闭环 0.025 m、无 >0.3 m 跳变）。
# ⚠️ B 收到 SIGINT 不落盘，彩色图只能靠 pcd.save_interval_sec:=15 的周期落盘取最后一份，
#    所以收尾必须等日志静止后再杀进程（下面 stable>=8 即 40s 无新增日志）。
set -o pipefail
REPO="${REPO:-$HOME/git/Super-LIVO}"
EXP=$REPO/experiments/octvox_ablation
TAG="${1:-move_hf_b}"
RESULTS="${RESULTS:-/mnt/dataset/slam_data/calib/results}"
OUT=$RESULTS/$TAG
BAG="${BAG:-/mnt/dataset/slam_data/calib/move_ros2}"
SLIO_CFG=$REPO/common/super_livo_common/config/lio_move.yaml
VIS_CFG=$EXP/20_move_b_node.yaml

export LD_LIBRARY_PATH="$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -vE 'trae|^\s*$' | paste -sd:)"
source /opt/ros/humble/setup.bash
source "$HOME/git/xtd2_ws/install/setup.bash" >/dev/null 2>&1 || true
source $REPO/third_party/Super-LIO/install/setup.bash
source $REPO/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

mkdir -p "$OUT"; rm -rf "$OUT"/*
pkill -x super_lio_node 2>/dev/null; pkill -x visual_b_node 2>/dev/null
pkill -f 'stamp_shift_relay|clock_shift_relay' 2>/dev/null; pkill -f 'odom_rec[o]der' 2>/dev/null
sleep 2

nohup python3 "$EXP/20_move_clock_shift_relay.py" > "$OUT/relay.log" 2>&1 &
nohup python3 $REPO/third_party/Super-LIO/exp/odom_recorder.py \
  --out "$OUT/odom.csv" --topic /lio/odom_shift > "$OUT/rec.log" 2>&1 &
ros2 run super_lio super_lio_node --ros-args \
  --params-file "$SLIO_CFG" \
  -p lio.map.save_map:=false -p lio.map.save_interval:=-1 \
  > "$OUT/slio.log" 2>&1 &
ros2 run super_livo_visual_b visual_b_node --ros-args \
  --params-file "$VIS_CFG" \
  -p pcd.save_dir:="$OUT" -p pcd.save_interval_sec:=15.0 \
  > "$OUT/vis.log" 2>&1 &

for i in $(seq 1 60); do grep -aq "Map init done" "$OUT/slio.log" && break; sleep 1; done
sleep 2

# 每 15s 抓一份彩色图，防止收尾阶段进程被杀导致整段结果丢失
( while true; do sleep 15
    f=$(grep -ao '/[^ ]*super_livo_visual_b_colored\.pcd' "$OUT/vis.log" 2>/dev/null | tail -1)
    [ -f "$f" ] && cp -f "$f" "$OUT/last_colored_backup.pcd" 2>/dev/null
  done ) & BAK=$!

# 本环境 DISPLAY 是 WSL 的 Xorg 桌面，rviz2 崩溃会把系统拖死；只观察、不处置
( while true; do sleep 60
    if [ -n "${DISPLAY:-}" ] && command -v xwdump >/dev/null 2>&1; then
      xwdump -m 2>/dev/null | tr -d '\r' > "$OUT/xwdump.txt"
    fi
    if grep -qaE 'Fatal Python error: could not rotate log file' "$OUT/xwdump.txt" 2>/dev/null; then
      grep -a 'Threads matching' -A 80 "$OUT/xwdump.txt" > "$OUT/CRASH_STACK_$(date +%H%M%S).txt"
    fi
    tail -1 "$OUT/vis.log" 2>/dev/null | cut -c1-160 >> "$OUT/watchdog.log"
  done ) & WD=$!

echo "=== [B] play bag (rate=${RATE:-0.05}) ==="
ros2 bag play "$BAG" --rate "${RATE:-0.05}" --disable-keyboard-controls < /dev/null > "$OUT/play.log" 2>&1
echo "=== [B] bag finished, wait visual catch-up (max 500s) ==="
prev=0; stable=0
for i in $(seq 1 100); do
  sleep 5
  cur=$(( $(wc -l < "$OUT/vis.log") + $(wc -l < "$OUT/slio.log") ))
  [ "$cur" = "$prev" ] && stable=$((stable+1)) || stable=0
  prev=$cur
  [ $stable -ge 8 ] && break
done
kill $WD $BAK 2>/dev/null
pkill -INT -x visual_b_node 2>/dev/null; sleep 15
pkill -x super_lio_node 2>/dev/null; pkill -x visual_b_node 2>/dev/null
pkill -f 'stamp_shift_relay|clock_shift_relay' 2>/dev/null; pkill -f 'odom_rec[o]der' 2>/dev/null
echo "=== [B] done ==="
grep -a "彩色地图已保存\|彩色统计" "$OUT/vis.log" | tail -2
ls -la "$OUT"
