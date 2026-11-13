#!/usr/bin/env bash
# 在 RTK-SLAM construction_seq2（已离线转换 /livox/lidar 类型）上跑 Super-LIO。
#   bash tools/run_rtk.sh            # 1x 实时播放，约 10 分钟
#   RATE=2 bash tools/run_rtk.sh     # 2x（仅用于快速冒烟，正式数字用 1x）
set -o pipefail

BAG=${BAG:-/home/ywj/slam_data/rtk_slam/construction_seq2_r2}
PARAMS=${PARAMS:-/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/livox_360.yaml}
OUT=${OUT:-/home/ywj/slam_data/rtk_slam/run_lio_seq2}
RATE=${RATE:-1.0}
TAG=${TAG:-seq2}          # 仅用于日志文件名，便于并行/连续跑多组
ITERS=${ITERS:-4}         # lio.kf.kf_max_iterations
BEAM=${BEAM:-0.0}         # lio.observe.beam_err（0=关闭）

# 同一时刻只允许一个 ROS 跑批（与 run_fl2_rtk.sh / run_ntu.sh 共用同一把锁）。
LOCK=/tmp/ros_batch.lock
exec 9>"$LOCK"
flock -n 9 || { echo "拒绝启动：$LOCK 已被另一个 ROS 跑批持有（禁止并行）"; exit 1; }
# 第二道闸：场上若有不持锁的跑批（如 /tmp/run_ntu_degen.sh）在跑，直接拒绝启动。
source /home/ywj/git/Super-LIVO/tools/ros_batch_guard.sh

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash     # 提供 livox_ros_driver2
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash

export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

pkill -x super_lio_node 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
rm -rf "$OUT"

ros2 run super_lio super_lio_node --ros-args --params-file "$PARAMS" \
  -p lio.sc_pgo.enable:=false -p lio.map.save_map:=false \
  -p lio.kf.kf_max_iterations:="$ITERS" \
  -p lio.observe.beam_err:="$BEAM" \
  > /tmp/slio_${TAG}.log 2>&1 &
SL=$!
sleep 6

ros2 bag record -o "$OUT" /lio/odom > /tmp/rec_${TAG}.log 2>&1 &
REC=$!
sleep 2

echo ">>> play $BAG @ ${RATE}x  iters=$ITERS  (PLAY_TIMEOUT=${PLAY_TIMEOUT:-none})"
if [ -n "$PLAY_TIMEOUT" ]; then
  timeout "$PLAY_TIMEOUT" ros2 bag play "$BAG" --rate "$RATE" --disable-keyboard-controls < /dev/null \
    > /tmp/play_${TAG}.log 2>&1
else
  ros2 bag play "$BAG" --rate "$RATE" --disable-keyboard-controls < /dev/null \
    > /tmp/play_${TAG}.log 2>&1
fi

sleep 3
kill -INT "$REC" 2>/dev/null; sleep 4; kill "$REC" 2>/dev/null
pkill -INT -x super_lio_node 2>/dev/null; sleep 10
pkill -x super_lio_node 2>/dev/null

echo "=== super_lio 日志尾部 ==="
tail -25 /tmp/slio_${TAG}.log
echo "=== odom 记录 ==="
ros2 bag info "$OUT" 2>&1 | sed -n '1,14p'
touch /tmp/rtk_${TAG}.done
