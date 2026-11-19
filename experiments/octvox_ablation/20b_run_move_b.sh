#!/usr/bin/env bash
# 方案 B（Super-LIO 主干 + visual_b_node 紧耦合）在 move_ros2 上的整段彩色重建（两进程模式）。
# 与 20a 同一 LIO 主干、同一 relay 时间域、同一外参，只换视觉支路。
# 用法: bash 20b_run_move_b.sh [tag]        默认 tag=move_hf_b，倍速 RATE=0.05
#
# ⚠️ 订正（2026-11-18，节点内 [PROBE-*] 探针实测，test4_move @RATE=1.0）：
#    B 不缺算力——单帧合计 avg 30 ms / 峰值 52 ms，其中前端 24 ms、约束装配 5.3 ms
#    （拷贝地图 0.2 + 投影建栅格 1.1 + LV 法方程 ~0 + 选轨 1.0 + MSCKF 2.4）、更新 0.4 ms、
#    上色 ~1 ms。9 Hz 图像只占约 0.27 个核，1× 实时毫无压力。
#    旧注释里的"单帧约 2.8 s / RATE>0.12 跑不完"是把 RCLCPP_INFO_THROTTLE(1000) 的
#    打印条数当成帧数得到的错读（"52 帧"同源于此），不可再用。
#    真正的丢帧来自等待与阻塞，不来自计算：828 图到达 → 513 帧处理，丢 315，其中
#    待定视觉队列超限（kMaxPendingVisualFrames=8）293、等位姿超时 19、位姿空洞 18；
#    触发原因是 odom 比图像晚 100~160 ms、图像流偶发 0.7~0.9 s 停顿，以及
#    pcd.save_interval_sec=15 的周期落盘单次阻塞 executor 855~980 ms。
# ⚠️ B 收到 SIGINT 不落盘，彩色图只能靠 pcd.save_interval_sec:=15 的周期落盘取最后一份，
#    所以收尾必须等日志静止后再杀进程（下面 stable>=8 即 40s 无新增日志）。
set -o pipefail
set -m   # 每个后台任务独立进程组 ⇒ kill -- -PID 能连 ros2 run 包装器一起收掉
REPO="${REPO:-$HOME/git/Super-LIVO}"
EXP=$REPO/experiments/octvox_ablation
TAG="${1:-move_hf_b}"
RESULTS="${RESULTS:-/mnt/dataset/slam_data/calib/results}"
OUT=$RESULTS/$TAG
BAG="${BAG:-/mnt/dataset/slam_data/calib/move_ros2}"
# 设备钟→主机钟偏移每个 bag 都不一样（上电重新对齐），必须显式给：
#   IMG_OFFSET_SEC = median(bag 录制时刻 - /livox/imu header)
: "${IMG_OFFSET_SEC:?该 bag 的设备钟→主机钟偏移，见 bag_probe 输出}"
export IMG_OFFSET_SEC
SLIO_CFG=$REPO/common/super_livo_common/config/lio_move.yaml
VIS_CFG=$EXP/20_move_b_node.yaml

export LD_LIBRARY_PATH="$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -vE 'trae|^\s*$' | paste -sd:)"
source /opt/ros/humble/setup.bash
source "$HOME/git/xtd2_ws/install/setup.bash" >/dev/null 2>&1 || true
source $REPO/third_party/Super-LIO/install/setup.bash
source $REPO/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# ⚠️ 不用 pkill：在 unshare 的 userns 里 pkill 不生效（上一轮因此留下孤儿 recorder，
#    把后续 run 的 odom 串写进同一个 CSV）。改成记录 PID + 按进程组回收。
PIDF=$OUT/pids
kpg(){ local p=$1 s=$2; [ -n "$p" ] && kill -"$s" -- "-$p" 2>/dev/null; return 0; }
mkdir -p "$OUT"
if [ -f "$PIDF" ]; then while read -r p; do kpg "$p" KILL; done < "$PIDF"; fi
rm -rf "$OUT"/*
for p in $(cat "$PIDF" 2>/dev/null); do kpg "$p" KILL; done; sleep 2

nohup python3 "$EXP/20_move_clock_shift_relay.py" > "$OUT/relay.log" 2>&1 &
RELAY_PID=$!
nohup python3 $REPO/third_party/Super-LIO/exp/odom_recorder.py \
  --out "$OUT/odom.csv" --topic /lio/odom_shift > "$OUT/rec.log" 2>&1 &
REC_PID=$!
ros2 run super_lio super_lio_node --ros-args \
  --params-file "$SLIO_CFG" \
  -p lio.map.save_map:=false -p lio.map.save_interval:=-1 \
  > "$OUT/slio.log" 2>&1 &
LIO_PID=$!
ros2 run super_livo_visual_b visual_b_node --ros-args \
  --params-file "$VIS_CFG" \
  -p super_livo_visual.pcd.save_dir:="$OUT" -p super_livo_visual.pcd.save_interval_sec:=15.0 \
  > "$OUT/vis.log" 2>&1 &
VIS_PID=$!
printf '%s\n%s\n%s\n%s\n' "$RELAY_PID" "$REC_PID" "$LIO_PID" "$VIS_PID" > "$PIDF"
echo "=== [B] pids relay=$RELAY_PID rec=$REC_PID lio=$LIO_PID vis=$VIS_PID ==="

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
  # [PROBE-*] 每 2s 刷新一次，会让"日志静止"永远不成立 ⇒ 统计行数时必须排除探针行
  c1=$(grep -avc PROBE "$OUT/vis.log" 2>/dev/null); c2=$(grep -avc PROBE "$OUT/slio.log" 2>/dev/null)
  cur=$(( ${c1:-0} + ${c2:-0} ))
  [ "$cur" = "$prev" ] && stable=$((stable+1)) || stable=0
  prev=$cur
  [ $stable -ge 8 ] && break
done
kill -9 "$WD" "$BAK" 2>/dev/null
kpg "$VIS_PID" INT; sleep 15
kpg "$LIO_PID" INT; sleep 5
for p in "$VIS_PID" "$LIO_PID" "$REC_PID" "$RELAY_PID"; do kpg "$p" KILL; done
sleep 2
for p in $(cat "$PIDF" 2>/dev/null); do
  kill -0 "$p" 2>/dev/null && { echo "WARN: pid $p 未退出，odom.csv 可能被串写"; kill -9 -- "-$p" 2>/dev/null; }
done
echo "[$TAG] recorder rows=$(wc -l < "$OUT/odom.csv")"
echo "=== [B] done ==="
grep -a "彩色地图已保存\|彩色统计" "$OUT/vis.log" | tail -2
ls -la "$OUT"
