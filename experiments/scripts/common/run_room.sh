#!/usr/bin/env bash
# 在 rosbag2_fastlivo2_2026_10_30-18_00_25（PointCloud2 版 Livox + 1280x800 相机）上跑
# Super-LIO 方案 A 对照。独立 ROS_DOMAIN_ID，不 pkill 其它跑批（避免误伤并行实验）。
#   bash /tmp/run_room.sh off      # 只跑 off
#   bash /tmp/run_room.sh on       # 只跑 on
#   SMOKE=25 bash /tmp/run_room.sh on   # 只播前 25 s 冒烟
set +u

CFG=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config
BAG=/home/ywj/slam_data/rosbag2_fastlivo2_2026_10_30-18_00_25
OUTROOT=/home/ywj/slam_data/room_slam
SMOKE=${SMOKE:-0}
mkdir -p "$OUTROOT"

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash   # livox_ros_driver2
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=77

run_one() {
  local tag=$1 cfg=$2
  local out=$OUTROOT/run_$tag
  local log=/tmp/slio_${tag}.log
  rm -rf "$out"; rm -f /tmp/room_${tag}.done
  ros2 run super_lio super_lio_node --ros-args --params-file "$cfg" \
    -p lio.sc_pgo.enable:=false -p lio.map.save_map:=false \
    > "$log" 2>&1 &
  local sl=$!
  sleep 6
  ros2 bag record -o "$out" /lio/odom > /tmp/rec_${tag}.log 2>&1 &
  local rec=$!
  sleep 2
  if [ "$SMOKE" -gt 0 ]; then
    timeout "$SMOKE" ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null \
      > /tmp/play_${tag}.log 2>&1
  else
    ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null \
      > /tmp/play_${tag}.log 2>&1
  fi
  sleep 3
  kill -INT "$rec" 2>/dev/null; sleep 4; kill "$rec" 2>/dev/null
  kill -INT "$sl" 2>/dev/null; sleep 8; kill -9 "$sl" 2>/dev/null
  touch /tmp/room_${tag}.done
  echo "=== $tag done"
  echo "--- [VisA] 行数: $(grep -c '\[VisA\]' "$log" || true)"
  echo "--- NaN 行数: $(grep -c 'nan' "$log" || true)"
  echo "--- 帧数统计:"
  grep -o 'first frame\|IMU Lag\|Degeneracy' "$log" | sort | uniq -c || true
  echo "--- 日志尾 8 行:"
  tail -8 "$log"
  echo "--- odom 记录:"
  ros2 bag info "$out" 2>&1 | sed -n '4,12p'
}

case "${1:-both}" in
  off) run_one room_off "$CFG/livox_360_visa_room_off.yaml" ;;
  on)  run_one room_on  "$CFG/livox_360_visa_room.yaml" ;;
  force) run_one room_force "$CFG/livox_360_visa_room_force.yaml" ;;
  force2) run_one room_force2 "$CFG/livox_360_visa_room_force2.yaml" ;;
  latch) run_one room_latch "$CFG/livox_360_visa_room_latch.yaml" ;;
  both)
    run_one room_off "$CFG/livox_360_visa_room_off.yaml"
    run_one room_on  "$CFG/livox_360_visa_room.yaml"
    ;;
esac
