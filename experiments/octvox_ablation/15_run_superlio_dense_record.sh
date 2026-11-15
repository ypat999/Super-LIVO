#!/usr/bin/env bash
# 只跑 Super-LIO（不做任何可视化/上色），录制 /lio/odom 与 /lio/cloud_body_dense，
# 供 15_superlio_official_color.py 离线按官方染色规则复算。
#
# 这样 A/B 节点代码零改动，且「位姿来源」与「染色规则」被彻底分开：
#   位姿 = Super-LIO（与 A/B 主链路同一份 retail_lio.yaml 参数）
#   染色 = 官方 publish_frame_world（无遮挡 / 无深度边缘 / 无 min_quality）
set -u
REPO=/home/ywj/git/Super-LIVO
BAG=${BAG:-/home/ywj/slam_data/Retail_Street}
OUT=${OUT:-/home/ywj/slam_data/superlio_dense_20261114}
mkdir -p "$OUT"

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-86}
export RCUTILS_LOGGING_BUFFERED_STREAM=1

set +u
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
set -u

# retail_lio.yaml 里的 app.* 只在 A/B 包装节点里声明，super_lio_node 不认识
sed '/^ *app\./d' "$REPO/config/retail_lio.yaml" > "$OUT/retail_lio_superlio.yaml"

pkill -x super_lio_node 2>/dev/null; sleep 2
rm -rf "$OUT/rec"

echo "[superlio] start node, out=$OUT"
ros2 run super_lio super_lio_node --ros-args \
  --params-file "$OUT/retail_lio_superlio.yaml" > "$OUT/slio.log" 2>&1 &
SL_PID=$!
sleep 5

ros2 bag record -o "$OUT/rec" /lio/odom /lio/cloud_body_dense \
  > "$OUT/record.log" 2>&1 &
REC_PID=$!
sleep 3

ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null \
  > "$OUT/play.log" 2>&1
echo "[superlio] bag 播完，收尾"
sleep 3
kill -INT $REC_PID 2>/dev/null; sleep 6
kill -INT $SL_PID 2>/dev/null; sleep 3
kill -9 $REC_PID 2>/dev/null; kill -9 $SL_PID 2>/dev/null

{
  echo "bag=$BAG"
  echo "out=$OUT"
  echo "start=$(date -Is)"
  ls -l "$OUT/rec" 2>/dev/null
} > "$OUT/meta.txt"
cat "$OUT/meta.txt"
echo "[superlio] done"
