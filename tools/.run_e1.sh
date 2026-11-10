#!/usr/bin/env bash
# E1：关闭跨帧颜色回填（median_voxel_size=0），其余与原 run_singleframe.sh
# 的 Super-LIVO 段完全一致（同 bag、同参数、同话题、同录制窗口）。
# 产物：slam_data/e1_append/{e1_map/*.pcd, e1_frames bag}
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
BAG=/home/ywj/slam_data/Retail_Street
REC_START=70
REC_LEN=25
OUT=/home/ywj/slam_data/e1_append
rm -rf "$OUT"; mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash
pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null; sleep 2
ros2 run super_lio super_lio_node --ros-args \
  --params-file /home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/r3live.yaml \
  > /tmp/e1_slio.log 2>&1 &
ros2 run super_livo_visual_a visual_a_node --ros-args \
  --params-file /home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml \
  -p pcd.save_dir:="$OUT/e1_map" -p pcd.save_interval_sec:=0.0 \
  -p pcd.median_voxel_size:=0.0 \
  > /tmp/e1_vis.log 2>&1 &
for i in $(seq 1 60); do grep -aq "Map init done" /tmp/e1_slio.log && break; sleep 1; done
sleep 2
ros2 bag play "$BAG" --rate 1.0 --disable-keyboard-controls < /dev/null > /tmp/e1_play.log 2>&1 &
P=$!
sleep "$REC_START"
ros2 bag record -o "$OUT/e1_frames" super_livo_visual_a/colored_cloud /lio/odom > "$OUT/e1_frames.rec.log" 2>&1 &
R=$!
sleep "$REC_LEN"
kill -INT $R 2>/dev/null; sleep 6; kill $R 2>/dev/null
sleep 2; kill $P 2>/dev/null
pkill -INT -x visual_a_node; sleep 50
pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null; sleep 3
grep -a "彩色地图已保存" /tmp/e1_vis.log | tail -1
grep -a "median\|时间中位数" /tmp/e1_vis.log | tail -2
ls -l "$OUT/e1_map/" 2>/dev/null
echo "===== E1 run done ====="
