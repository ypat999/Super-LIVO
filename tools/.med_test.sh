#!/usr/bin/env bash
set -o pipefail
SL_CFG=${SL_CFG:-/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/r3live.yaml}
A_CFG=${A_CFG:-/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml}
BAG=/home/ywj/slam_data/Retail_Street
# 体素边长（米）与输出目录可通过环境变量覆盖，默认 1cm / median_test
VOX=${VOX:-0.01}
TAG=${TAG:-med}
OUT=${OUT:-/home/ywj/slam_data/pcd/median_test}
# 非空则同时录 /lio/odom 并出轨迹对比（用于地图表示消融）
ODOM_OUT=${ODOM_OUT:-}

# WSL 下固定 DDS 传输，避免 discovery 超时导致启动慢/丢消息
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash

pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null
sleep 2
rm -rf "$OUT"; mkdir -p "$OUT"

ros2 run super_lio super_lio_node --ros-args --params-file "$SL_CFG" > /tmp/slio_$TAG.log 2>&1 &
ros2 run super_livo_visual_a visual_a_node --ros-args --params-file "$A_CFG" \
  -p pcd.save_dir:="$OUT" -p pcd.save_interval_sec:=0.0 \
  -p pcd.median_voxel_size:="$VOX" \
  > /tmp/vis_$TAG.log 2>&1 &

# 等 Super-LIO 完成地图初始化（最多 60 s），避免 bag 已开播而 LIO 未就绪
for i in $(seq 1 60); do
  grep -aq "Map init done" /tmp/slio_$TAG.log 2>/dev/null && break
  sleep 1
done
sleep 2

T0=$(date +%s)
if [ -n "$ODOM_OUT" ]; then
  rm -rf "$ODOM_OUT"
  ros2 bag record -o "$ODOM_OUT" /lio/odom > /tmp/rec_$TAG.log 2>&1 &
  REC=$!
  sleep 2
fi
ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null > /tmp/bagplay_$TAG.log 2>&1
T1=$(date +%s)
echo "bag 播放耗时: $((T1 - T0)) s" >> /tmp/vis_$TAG.log

if [ -n "$ODOM_OUT" ]; then
  sleep 3
  kill -INT $REC 2>/dev/null; sleep 4; kill $REC 2>/dev/null
  python3 /tmp/odom2tum.py "$ODOM_OUT" /lio/odom "$ODOM_OUT.txt" >> /tmp/vis_$TAG.log 2>&1
  python3 /home/ywj/git/Super-LIVO/tools/traj_compare.py \
    /home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_poses.txt \
    "$ODOM_OUT.txt" > "/tmp/traj_$TAG.txt" 2>&1
  grep -a -e 位置差 -e 旋转差 -e 位移差 "/tmp/traj_$TAG.txt"
fi

sleep 3
# 退出时会把体素中位色回填到全部几何点（千万级哈希查询 + 写盘），等到进程真正退出
pkill -INT -x visual_a_node
for i in $(seq 1 180); do
  pgrep -x visual_a_node > /dev/null || break
  sleep 2
done
pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null

echo "=== done VOX=$VOX bag=$((T1 - T0))s ==="
grep -a "彩色地图已保存" /tmp/vis_$TAG.log | tail -1
grep -a "彩色统计" /tmp/vis_$TAG.log | tail -1
grep -a "视觉帧" /tmp/vis_$TAG.log | tail -1
ls -lh "$OUT"
