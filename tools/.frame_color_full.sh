#!/usr/bin/env bash
# 纯 Super-LIO 全量跑 Retail_Street，逐帧导出世界系稠密点云（scans_*.pcd），
# 供 frame_id_color_map 按帧染色。不改任何算法：只是打开既有 save_map 开关。
set -o pipefail
SL_CFG=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/r3live.yaml
BAG=/home/ywj/slam_data/Retail_Street
OUT=/home/ywj/slam_data/pcd/retail_framecolor

export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash

pkill -x super_lio_node 2>/dev/null
sleep 2
rm -rf "$OUT"

ros2 run super_lio super_lio_node --ros-args --params-file "$SL_CFG" \
  -p lio.map.save_map:=true \
  -p lio.map.save_interval:=1 \
  -p lio.map.save_map_dir:="$OUT" \
  > /tmp/slio_fc.log 2>&1 &

for i in $(seq 1 60); do
  grep -aq "Map init done" /tmp/slio_fc.log 2>/dev/null && break
  sleep 1
done
sleep 2

T0=$(date +%s)
ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null > /tmp/bagplay_fc.log 2>&1
echo "bag 播放耗时: $(($(date +%s) - T0)) s"

# 等异步 dense/save 队列排空：扫描文件数连续 15s 不变即认为写完
prev=0
for i in $(seq 1 60); do
  sleep 5
  cur=$(ls "$OUT"/PCD/scans_*.pcd 2>/dev/null | wc -l)
  echo "scans=$cur"
  [ "$cur" -gt 100 ] && [ "$cur" -eq "$prev" ] && break
  prev=$cur
done
pkill -x super_lio_node 2>/dev/null
echo "=== done: $(ls "$OUT"/PCD/scans_*.pcd | wc -l) 帧 ==="
du -sh "$OUT"/PCD
