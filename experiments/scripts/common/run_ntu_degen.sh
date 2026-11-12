#!/usr/bin/env bash
# 退化场景跑批：用 cloud_crop.py 把 LiDAR 点云裁成纯平面/走廊（相机不动），
# Super-LIO 通过 remap 订阅裁剪后的点云，/lio/degeneracy 会真实升高。
#
#   SIDE=lio TAG=d1 MODE=ground Z_HALF=0.3 bash /tmp/run_ntu_degen.sh
#   SIDE=b   TAG=d1 MODE=ground Z_HALF=0.3 bash /tmp/run_ntu_degen.sh
set -o pipefail

SEQ=${SEQ:-eee_03}
SIDE=${1:-lio}
RATE=${RATE:-0.5}
TAG=${TAG:-d1}
MODE=${MODE:-ground}
export MODE Z_HALF=${Z_HALF:-0.3} Z_CENTER=${Z_CENTER:-0.0} Y_HALF=${Y_HALF:-1.2} START_OFFSET=${START_OFFSET:-0}
# 限时播放（秒，wall clock）。humble 的 ros2 bag play 没有"只播到某时刻"的参数，
# 需要短窗口扫描时用 timeout 掐掉，避免每轮都跑满 181s/rate。
PLAY_TIMEOUT=${PLAY_TIMEOUT:-0}
BAG=/home/ywj/slam_data/ntu_viral/bags2/$SEQ
SL_NTU=${SL_NTU:-/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/NTU.yaml}
ODOMDIR=/home/ywj/slam_data/ntu_odom_${SEQ}_degen_${SIDE}_${TAG}

# ---- 纳入全局跑批锁（与 tools/run_fl2_rtk.sh / run_ntu.sh / run_rtk.sh 同一把）----
# 本脚本原本不持锁，而 tools 脚本开头有 pkill -x super_lio_node / -x visual_b_node /
# -f image_transport/republish，只要 tools 那侧 flock 成功就会把本进程组正在用的
# 节点打死（实测 dm6guard 那轮 odom bag 只剩 24 KB）。
# PARALLEL_OK=1 是显式的"degen 成对并跑"（同一 DOMAIN，收尾只用 $! 精确杀），
# 这种跑法不能占锁；反向的保护由 tools 侧 ros_batch_guard.sh 负责：它见到场上有
# 活节点就拒绝启动，绝不去杀。
LOCK=/tmp/ros_batch.lock
if [ "${PARALLEL_OK:-0}" = "1" ]; then
  echo 'PARALLEL_OK=1：跳过 '"$LOCK"'（成对并跑），收尾只用 $! 精确杀'
else
  exec 9>"$LOCK"
  flock -n 9 || { echo "拒绝启动：$LOCK 已被另一个 ROS 跑批持有（禁止并行）"; exit 1; }
  source /home/ywj/git/Super-LIVO/tools/ros_batch_guard.sh
fi

export ROS_DOMAIN_ID=${DOMAIN:-37}

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash

if [ "${PARALLEL_OK:-0}" != "1" ] && { pgrep -x visual_b_node >/dev/null || pgrep -x super_lio_node >/dev/null \
   || pgrep -f cloud_crop.py >/dev/null; }; then
  echo "拒绝启动：仍有残留节点"; pgrep -a -f 'visual_b_node|super_lio_node|cloud_crop'; exit 1
fi

rm -rf "$ODOMDIR"

python3 /tmp/cloud_crop.py > /tmp/crop_${SEQ}_${TAG}.log 2>&1 &
CROP=$!

ros2 run super_lio super_lio_node --ros-args --params-file "$SL_NTU" \
  -p lio.observe.degeneracy_detect_en:=true -p lio.observe.degen_trans_only:=1 \
  -r /os1_cloud_node1/points:=/degen/points \
  > /tmp/slio_${SEQ}_degen_${TAG}.log 2>&1 &
SLIO=$!

if [ "$SIDE" = "b" ]; then
  ros2 run super_livo_visual_b visual_b_node --ros-args \
    --params-file /home/ywj/git/Super-LIVO/solution_b_tight_coupled/super_livo_visual_b/config/visual_b_ntu.yaml \
    $EXTRA > /tmp/vis_degen_${TAG}.log 2>&1 &
  VIS=$!
  REC_TOPICS="/lio/odom /lio/degeneracy /super_livo_visual_b/odom"
elif [ "$SIDE" = "a" ]; then
  ros2 run super_livo_visual_a visual_a_node --ros-args \
    --params-file /home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_ntu.yaml \
    $EXTRA > /tmp/vis_degen_${TAG}.log 2>&1 &
  VIS=$!
  REC_TOPICS="/lio/odom /lio/degeneracy /super_livo_visual_a/odom"
else
  REC_TOPICS="/lio/odom /lio/degeneracy"
fi

sleep 8
ros2 bag record -o "$ODOMDIR" $REC_TOPICS > /tmp/rec_${SEQ}_degen_${TAG}.log 2>&1 &
REC=$!
sleep 2
if [ "$PLAY_TIMEOUT" != "0" ]; then
  timeout -s INT "$PLAY_TIMEOUT" ros2 bag play "$BAG" --rate "$RATE" $PLAY_EXTRA \
    > /tmp/bagplay_${SEQ}_degen_${TAG}.log 2>&1
else
  ros2 bag play "$BAG" --rate "$RATE" $PLAY_EXTRA > /tmp/bagplay_${SEQ}_degen_${TAG}.log 2>&1
fi
sleep 3
kill -INT $REC 2>/dev/null; sleep 3; kill -9 $REC 2>/dev/null
# 只杀自己这一路：先按父子关系杀 ros2 run 的真实节点子进程，再杀 wrapper。
# 顺序反了会把节点孤儿化（PPID 变成 1），后面按 -P 就再也找不到它。
# 原先这里是 pkill -9 -x visual_b_node / super_lio_node 全局按名字杀，
# 并行跑另一路 degen 时正好会打死对方。
for _w in "$VIS" "$SLIO"; do
  [ -n "$_w" ] || continue
  pkill -INT -P "$_w" 2>/dev/null; kill -INT "$_w" 2>/dev/null
done
kill -INT $CROP 2>/dev/null
sleep 8
for _w in "$VIS" "$SLIO"; do
  [ -n "$_w" ] || continue
  pkill -9 -P "$_w" 2>/dev/null; kill -9 "$_w" 2>/dev/null
done
kill -9 $CROP 2>/dev/null
sleep 1

echo "=== 收尾 ==="
tail -2 /tmp/crop_${SEQ}_${TAG}.log
grep -a "退化\|degen" /tmp/slio_${SEQ}_degen_${TAG}.log | tail -5
echo "ODOMDIR=$ODOMDIR"
