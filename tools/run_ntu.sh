#!/usr/bin/env bash
# NTU VIRAL 跑批：Super-LIO + 方案A/B（或官方 FAST-LIVO2），实时播放 + 录制 odom + SIGINT 收尾落盘。
#
#   SEQ=eee_03 bash tools/run_ntu.sh a     # Super-LIO + 方案A
#   SEQ=eee_03 bash tools/run_ntu.sh b     # Super-LIO + 方案B
#   SEQ=eee_03 bash tools/run_ntu.sh fl    # 官方 FAST-LIVO2（位姿由其自身文件输出）
#
# 评测：
#   python3 tools/odom_vs_gt.py --gt ntu_viral/gt/$SEQ/ground_truth.csv \
#       --bag ~/slam_data/ntu_odom_${SEQ}_a --topic /lio/odom --label "lio/odom" \
#       --bag ~/slam_data/ntu_odom_${SEQ}_a --topic /super_livo_visual_a/odom --label "方案A"
set -o pipefail  # 不用 -u：ROS 的 setup.bash 会引用未定义变量

SEQ=${SEQ:-eee_03}
SIDE=${1:-a}
BAG=/home/ywj/slam_data/ntu_viral/bags2/$SEQ
SL_NTU=${SL_NTU:-/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/NTU.yaml}
WS_FL=/home/ywj/fastlivo2_ws
FL_CFG=/home/ywj/git/Super-LIVO/third_party/FAST-LIVO2-ROS2/config
GT=/home/ywj/slam_data/ntu_viral/gt/$SEQ/ground_truth.csv
ODOMDIR=/home/ywj/slam_data/ntu_odom_${SEQ}_${SIDE}

[ -d "$BAG" ] || { echo "缺少转换后的 bag: $BAG（先跑 tools/ntuviral_prep.sh --seq $SEQ）"; exit 1; }
[ -f "$GT" ] || echo "警告: 真值缺失 $GT"

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
source /tmp/sl_install/setup.bash

pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null
pkill -x visual_b_node 2>/dev/null; pkill -f fastlivo_mapping 2>/dev/null
pkill -f parameter_blackboard 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
rm -rf "$ODOMDIR"

if [ "$SIDE" = "fl" ]; then
  source $WS_FL/install/setup.bash
  ros2 run demo_nodes_cpp parameter_blackboard --ros-args \
    --params-file "$FL_CFG/camera_NTU_VIRAL.yaml" > /tmp/fl_bb_${SEQ}.log 2>&1 &
  BB=$!
  sleep 8
  # IMG_EN=0 关掉视觉，只跑官方 LIO（消融对照）
  IMG_EN=${IMG_EN:-1}
  $WS_FL/install/fast_livo/lib/fast_livo/fastlivo_mapping --ros-args \
    --params-file "$FL_CFG/NTU_VIRAL.yaml" --params-file "$FL_CFG/camera_NTU_VIRAL.yaml" \
    -p evo.seq_name:="${SEQ}${FL_TAG:-}" -p pcd_save.pcd_save_en:=false -p pcd_save.interval:=-1 \
    -p common.img_en:="$IMG_EN" \
    > /tmp/fl_${SEQ}_${SIDE}.log 2>&1 &
  NODE_PID=$!
  sleep 12
  rm -rf "$ODOMDIR"
  ros2 bag record -o "$ODOMDIR" /aft_mapped_to_init \
    > /tmp/rec_${SEQ}_${SIDE}.log 2>&1 &
  REC=$!
  ros2 bag play "$BAG" --rate 1.0 > /tmp/bagplay_${SEQ}_${SIDE}.log 2>&1
  sleep 5
  kill -INT $REC 2>/dev/null; sleep 4; kill $REC 2>/dev/null
  kill -INT $NODE_PID 2>/dev/null; sleep 8
  kill $BB 2>/dev/null
  pkill -f fastlivo_mapping 2>/dev/null
  echo "=== 官方收尾（位姿文件见 third_party/FAST-LIVO2-ROS2/Log）==="
  tail -3 /tmp/fl_${SEQ}_${SIDE}.log
  find /home/ywj/git/Super-LIVO/third_party/FAST-LIVO2-ROS2/Log -name "*.txt" -newermt "-10 minutes" 2>/dev/null | head
  echo "评测：python3 tools/odom_vs_gt.py --gt /home/ywj/slam_data/ntu_viral/gt/$SEQ/ground_truth.csv --bag ${ODOMDIR} --topic /aft_mapped_to_init --label 官方FAST-LIVO2 --out /tmp/ntu_${SEQ}_fl.png"
  exit 0
fi

if [ "$SIDE" = "a" ]; then
  NODE_PKG=super_livo_visual_a; NODE_EXE=visual_a_node
  PARAMS=/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_ntu.yaml
  OWN=/super_livo_visual_a/odom
else
  NODE_PKG=super_livo_visual_b; NODE_EXE=visual_b_node
  PARAMS=/home/ywj/git/Super-LIVO/solution_b_tight_coupled/super_livo_visual_b/config/visual_b_ntu.yaml
  OWN=/super_livo_visual_b/odom
fi
VLOG=/tmp/vis_${SIDE}_${SEQ}.log

ros2 run super_lio super_lio_node --ros-args --params-file "$SL_NTU" > /tmp/slio_${SEQ}_${SIDE}.log 2>&1 &
ros2 run "$NODE_PKG" "$NODE_EXE" --ros-args --params-file "$PARAMS" > "$VLOG" 2>&1 &
sleep 6
ros2 bag record -o "$ODOMDIR" /lio/odom /lio/cloud_body_dense "$OWN" \
  > /tmp/rec_${SEQ}_${SIDE}.log 2>&1 &
REC=$!
sleep 2
ros2 bag play "$BAG" --rate 1.0 > /tmp/bagplay_${SEQ}_${SIDE}.log 2>&1
sleep 3
kill -INT $REC 2>/dev/null; sleep 3; kill $REC 2>/dev/null
pkill -INT -x "$NODE_EXE"; sleep 8
pkill -x super_lio_node 2>/dev/null; pkill -x "$NODE_EXE" 2>/dev/null

echo "=== $NODE_EXE 收尾 ==="
grep -a "彩色统计" "$VLOG" | tail -1
tail -2 "$VLOG"
ls -l /home/ywj/slam_data/pcd/ntu_${SIDE}/*.pcd 2>/dev/null
echo
echo "评测：python3 tools/odom_vs_gt.py --gt $GT --bag $ODOMDIR --topic /lio/odom --label lio/odom \\"
echo "        --bag $ODOMDIR --topic $OWN --label 方案${SIDE^^} --out /tmp/ntu_${SEQ}_${SIDE}.png"
