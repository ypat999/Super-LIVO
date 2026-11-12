#!/usr/bin/env bash
# B2 冒烟/正式跑批：bash /tmp/run_b2.sh <tag> <cfg.yaml> [SMOKE秒数] [额外 ros2 参数...]
set +u

BAG=/home/ywj/slam_data/rosbag2_fastlivo2_2026_10_30-18_00_25
OUTROOT=/home/ywj/slam_data/room_slam

TAG=$1
CFG=$2
SMOKE=${3:-0}
shift 3 2>/dev/null
EXTRA="$*"

mkdir -p "$OUTROOT"
source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=77

out=$OUTROOT/run_$TAG
log=/tmp/slio_${TAG}.log
rm -rf "$out"; rm -f /tmp/room_${TAG}.done
ros2 run super_lio super_lio_node --ros-args --params-file "$CFG" \
  -p lio.sc_pgo.enable:=false -p lio.map.save_map:=false $EXTRA \
  > "$log" 2>&1 &
sl=$!
sleep 6
ros2 bag record -o "$out" /lio/odom > /tmp/rec_${TAG}.log 2>&1 &
rec=$!
sleep 2
if [ "$SMOKE" -gt 0 ]; then
  timeout "$SMOKE" ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null \
    > /tmp/play_${TAG}.log 2>&1
else
  ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null \
    > /tmp/play_${TAG}.log 2>&1
fi
sleep 3
kill -INT "$rec" 2>/dev/null; sleep 4; kill "$rec" 2>/dev/null
kill -INT "$sl" 2>/dev/null; sleep 8; kill -9 "$sl" 2>/dev/null
touch /tmp/room_${TAG}.done

echo "=== $TAG done (SMOKE=$SMOKE EXTRA=$EXTRA)"
echo "--- 日志行数: $(wc -l < "$log")  VisRot 行数: $(grep -c '\[VisRot\]' "$log" || true)"
echo "--- 错误行:"
grep -iE 'error|what\(\)|Segmentation|nan|Exception|FATAL' "$log" | head -10
echo "--- 订阅与启用:"
grep -E 'VisRot\] (图像订阅|视觉相对旋转)' "$log" | head -4
echo "--- 最后 4 条 VisRot:"
grep '\[VisRot\]' "$log" | tail -4
echo "--- odom 记录:"
ros2 bag info "$out" 2>&1 | sed -n '4,12p'
