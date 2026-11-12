#!/bin/bash
# B3 阶段1：room bag
#   A1 b3off  : 新构建 + enable=false，验收1 与 run_room_off 逐位比对
#   A2 b3smoke: B3 smoke(40s)，取 FD 校核日志
set +u
BAG=/home/ywj/slam_data/rosbag2_fastlivo2_2026_10_30-18_00_25
OUTROOT=/home/ywj/slam_data/room_slam
CFGDIR=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config

exec 9>/tmp/ros_batch.lock
flock -n 9 || { echo "锁被占用"; exit 9; }

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=77

run_one() {
  local TAG=$1 CFG=$2 SMOKE=${3:-0}; shift 3
  local EXTRA="$*"
  local out=$OUTROOT/run_$TAG log=/tmp/slio_$TAG.log
  rm -rf "$out"; rm -f /tmp/room_$TAG.done
  ros2 run super_lio super_lio_node --ros-args --params-file "$CFG" \
    -p lio.sc_pgo.enable:=false -p lio.map.save_map:=false $EXTRA \
    > "$log" 2>&1 &
  local sl=$!
  sleep 6
  ros2 bag record -o "$out" /lio/odom > /tmp/rec_$TAG.log 2>&1 &
  local rec=$!
  sleep 2
  if [ "$SMOKE" -gt 0 ]; then
    timeout "$SMOKE" ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null > /tmp/play_$TAG.log 2>&1
  else
    ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null > /tmp/play_$TAG.log 2>&1
  fi
  sleep 3
  kill -INT "$rec" 2>/dev/null; sleep 4; kill "$rec" 2>/dev/null
  kill -INT "$sl" 2>/dev/null; sleep 8; kill -9 "$sl" 2>/dev/null
  touch /tmp/room_$TAG.done
  echo "=== $TAG done SMOKE=$SMOKE EXTRA=$EXTRA"
  grep -iE 'error|Segmentation|Exception|FATAL' "$log" | head -5
  ros2 bag info "$out" 2>&1 | grep -E 'Messages:|Duration:'
}

echo "===== A1: b3off (enable=false) ====="
run_one b3off $CFGDIR/livox_360_visa_room_b3.yaml 0 -p lio.visual.enable:=false
echo "===== bitcmp vs run_room_off ====="
python3 /tmp/b3_bitcmp.py $OUTROOT/run_b3off $OUTROOT/run_room_off
echo "===== A2: b3smoke (B3 开, 40s) ====="
run_one b3smoke $CFGDIR/livox_360_visa_room_b3.yaml 40
echo "===== FD/VisB3 日志 ====="
grep -a "VisB3bg" /tmp/slio_b3smoke.log | head -12
grep -a "VisB3" /tmp/slio_b3smoke.log | grep -av VisB3bg | tail -8
echo BATCH1-DONE
