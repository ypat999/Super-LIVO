#!/usr/bin/env bash
# 方案 A（Super-LIO 主干 + visual_a_node 上色）在 move_ros2 上的整段彩色重建（两进程模式）。
# 图像 header 是主机时钟、LIO 是设备时钟；用 20_move_clock_shift_relay.py 把 LIO 侧
# odom/点云抬到主机时钟域（只平移这两个话题，绝不平移 /tf，否则外参推相机中心全错）。
# 用法: bash 20a_run_move_a.sh [tag]        默认 tag=move_hf_a，倍速 RATE=0.6
# 实测（2026-11-17 holefix 外参，RATE=0.6）：76.4s 数据 → 739 帧、path 9.581 m、闭环 0.027 m、
# 彩色图 3.38 M 点（721/741 帧着色，无 >0.3 m 跳变）。
set -o pipefail
set -m   # 每个后台任务独立进程组 ⇒ kill -- -PID 能连 ros2 run 包装器一起收掉
REPO="${REPO:-$HOME/git/Super-LIVO}"
EXP=$REPO/experiments/octvox_ablation
TAG="${1:-move_hf_a}"
RESULTS="${RESULTS:-/mnt/dataset/slam_data/calib/results}"
OUT=$RESULTS/$TAG
BAG="${BAG:-/mnt/dataset/slam_data/calib/move_ros2}"
# 设备钟→主机钟偏移每个 bag 都不一样（上电重新对齐），必须显式给：
#   IMG_OFFSET_SEC = median(bag 录制时刻 - /livox/imu header)
: "${IMG_OFFSET_SEC:?该 bag 的设备钟→主机钟偏移，见 /tmp/bag_probe.py 或 bag_probe 输出}"
export IMG_OFFSET_SEC
SLIO_CFG=$REPO/common/super_livo_common/config/lio_move.yaml
VIS_CFG=$EXP/20_move_a_node.yaml

export LD_LIBRARY_PATH="$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -vE 'trae|^\s*$' | paste -sd:)"
source /opt/ros/humble/setup.bash
source "$HOME/git/xtd2_ws/install/setup.bash" >/dev/null 2>&1 || true
source $REPO/third_party/Super-LIO/install/setup.bash
source $REPO/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# ⚠️ 不用 pkill：在 unshare 的 userns 里 pkill 不生效，上一轮就是因此留下 9 个孤儿
#    recorder，把后面所有 run 的 odom 串写进同一个 CSV。改成记录 PID + 按进程组回收，
#    PID 落盘到 $OUT/pids，下次进同 tag 时先按上次的清单收干净（在清目录之前读）。
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
ros2 run super_livo_visual_a visual_a_node --ros-args \
  --params-file "$VIS_CFG" \
  -p super_livo_visual.pcd.save_dir:="$OUT" -p super_livo_visual.pcd.save_interval_sec:=0.0 \
  > "$OUT/vis.log" 2>&1 &
VIS_PID=$!
printf '%s\n%s\n%s\n%s\n' "$RELAY_PID" "$REC_PID" "$LIO_PID" "$VIS_PID" > "$PIDF"
echo "=== [A] pids relay=$RELAY_PID rec=$REC_PID lio=$LIO_PID vis=$VIS_PID ==="

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
kpg "$VIS_PID" INT; sleep 50                            # SIGINT 后 A 会落盘彩色地图，必须等
kpg "$LIO_PID" INT; sleep 5                              # 让 LIO 走完退出路径
for p in "$VIS_PID" "$LIO_PID" "$REC_PID" "$RELAY_PID"; do kpg "$p" KILL; done
sleep 2
for p in $(cat "$PIDF" 2>/dev/null); do
  kill -0 "$p" 2>/dev/null && { echo "WARN: pid $p 未退出，odom.csv 可能被串写"; kill -9 -- "-$p" 2>/dev/null; }
done
echo "[$TAG] recorder rows=$(wc -l < "$OUT/odom.csv")"
grep -a "彩色地图已保存\|彩色统计" "$OUT/vis.log" | tail -2
ls -la "$OUT"
