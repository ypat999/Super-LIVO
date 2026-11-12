#!/usr/bin/env bash
# 决定性实验（§7.3）单次跑批：room bag，指定模式与 IMU 噪声，debug 级别可控。
# 隔离：独立 ROS_DOMAIN_ID=78（避免与并行 rpb.sh 的 drone 回放撞话题）；
#       pre_clean 只杀本实验（room_b3.yaml / run_dec_*）进程，不误伤其它跑批。
# 用法: bash /tmp/deci_one.sh <tag> <mode: off|b2> <imu_nbg> <imu_ng> <debug>
set +u
TAG=$1; MODE=$2; NBG=$3; NG=$4; DBG=$5; SIG=${6:-1e-3}
BAG=/home/ywj/slam_data/rosbag2_fastlivo2_2026_10_30-18_00_25
OUTROOT=/home/ywj/slam_data/room_slam
Y=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/livox_360_visa_room_b3.yaml

exec 9>/tmp/ros_batch.lock
flock 9

pre_clean() {
  for p in $(pgrep -f 'super_lio_node --ros-args.*livox_360_visa_room_b3.yaml'); do kill -9 "$p" 2>/dev/null; done
  for p in $(pgrep -f 'ros2 bag (record|play).*room_slam/run_dec_|ros2 bag play.*rosbag2_fastlivo2_2026_10_30'); do kill -9 "$p" 2>/dev/null; done
  sleep 2
}
pre_clean

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=78

case "$MODE" in
  off) BASE="-p lio.visual.enable:=false" ;;
  b2)  BASE="-p lio.visual.enable:=true -p lio.visual.b2.enable:=true -p lio.visual.b3.enable:=false -p lio.visual.sigma_rot:=$SIG -p lio.visual.chi2_thresh:=9.0" ;;
esac
NOISE="-p lio.sensor.imu_nbg:=$NBG -p lio.sensor.imu_ng:=$NG"

out=$OUTROOT/run_$TAG
log=/tmp/slio_${TAG}.log
rm -rf "$out"
ros2 run super_lio super_lio_node --ros-args --params-file "$Y" \
  -p lio.sc_pgo.enable:=false -p lio.map.save_map:=false \
  -p lio.debug.level:=$DBG $BASE $NOISE > "$log" 2>&1 &
sl=$!
sleep 6
ros2 bag record -o "$out" /lio/odom > /tmp/rec_${TAG}.log 2>&1 &
rec=$!
sleep 2
ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null > /tmp/play_${TAG}.log 2>&1
sleep 3
kill -INT "$rec" 2>/dev/null; sleep 4; kill "$rec" 2>/dev/null
kill -INT "$sl" 2>/dev/null; sleep 8; kill -9 "$sl" 2>/dev/null
pre_clean

echo "### RESULT $TAG mode=$MODE nbg=$NBG ng=$NG dbg=$DBG"
grep -a '\[Param\] debug/level' "$log" | head -1
grep -a '\[Param\] visual/b2' "$log" | head -1
grep -a '\[Param\]' "$log" | grep -E 'imu_nbg|imu_ng' | head -2
python3 /tmp/odom_metrics.py "$out" "$TAG"
if [ "$MODE" != "off" ]; then
  echo "  ACC_END: $(grep -a '\[VisRot\] inl=' "$log" | tail -1 | grep -o 'acc=[^(]*([^)]*)')"
  echo "  GATE: $(grep -a '\[VisB2gate\]' "$log" | tail -1 | cut -c1-250)"
  echo "  REJ: $(grep -ac '\[VisB2\] reject' "$log")  VISFRAMES: $(grep -ac '\[VisRot\] inl=' "$log")"
  python3 /tmp/acc_vs_p.py "$log"
fi
echo "### DONE $TAG"
