#!/usr/bin/env bash
# 阶段二：完整 LIO 闭环，A/B/C 三个独立运行实例。
# 唯一差异 = lio.hash_map.rep_mode（0=增量均值 A，2=首次观测 B，1=最近中心 C）。
# 其余全部沿用 r3live.yaml 原文件（不改配置，用 -p 覆盖），
# 与历史重放 replay3 的启动方式一致。
set -o pipefail
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
# livox_ros_driver2（CustomMsg 类型支持）必须先于 Super-LIO 载入，与历史 replay3 一致
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash 2>/dev/null || true

CFG=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/r3live.yaml
BAG=/home/ywj/slam_data/Retail_Street
OUT=/home/ywj/slam_data/octvox_closedloop_20261110
QOS=/tmp/rec_qos.yaml
mkdir -p "$OUT"

for pair in A:0 B:2 C:1; do
  name="${pair%%:*}"; mode="${pair##*:}"
  dir="$OUT/rep_$name"
  rm -rf "$dir"; mkdir -p "$dir"
  echo "=================== RUN $name (rep_mode=$mode) ==================="
  pkill -x super_lio_node 2>/dev/null; sleep 2
  /usr/bin/time -v -o "$dir/time.txt" bash -c \
    "exec ros2 run super_lio super_lio_node --ros-args \
      --params-file $CFG \
      -p lio.hash_map.rep_mode:=$mode \
      -p lio.debug.level:=2 \
      -p lio.eva.timer:=true \
      -p lio.map.save_map_dir:=$dir" \
    > "$dir/node.log" 2>&1 &
  sleep 6
  if ! pgrep -x super_lio_node > /dev/null; then
    echo "!!! $name: node 未启动，见 $dir/node.log"; continue
  fi
  ros2 bag record -o "$dir/bag" --qos-profile-overrides-path "$QOS" /lio/odom \
    > "$dir/rec.log" 2>&1 &
  REC=$!
  sleep 2
  ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null \
    > "$dir/play.log" 2>&1
  sleep 3
  kill -INT "$REC"; sleep 6; kill "$REC" 2>/dev/null
  pkill -x super_lio_node 2>/dev/null; sleep 3
  echo "--- $name done: $(wc -l < "$dir/debug.csv" 2>/dev/null) debug rows"
done
echo DONE_CLOSEDLOOP
