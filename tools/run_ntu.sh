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
# 播放速率。默认 1.0；Super-LIO 单帧观测在本机约 140 ms（> 100 ms 扫描间隔）时会积压，
# 触发 [odom queue] 积压丢帧，lio/odom 在中途断流（实测 180 s 的 bag 只输出 53 s）。
# 需要完整里程计时用 RATE=0.5 跑。
RATE=${RATE:-1.0}
BAG=/home/ywj/slam_data/ntu_viral/bags2/$SEQ
SL_NTU=${SL_NTU:-/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/NTU.yaml}
WS_FL=/home/ywj/fastlivo2_ws
FL_CFG=/home/ywj/git/Super-LIVO/third_party/FAST-LIVO2-ROS2/config
GT=/home/ywj/slam_data/ntu_viral/gt/$SEQ/ground_truth.csv
ODOMDIR=/home/ywj/slam_data/ntu_odom_${SEQ}_${SIDE}

[ -d "$BAG" ] || { echo "缺少转换后的 bag: $BAG（先跑 tools/ntuviral_prep.sh --seq $SEQ）"; exit 1; }
[ -f "$GT" ] || echo "警告: 真值缺失 $GT"

# 同一时刻只允许一个 ROS 跑批（与 run_fl2_rtk.sh / run_rtk.sh 共用同一把锁）。
# 本脚本开头的 pkill 会打死别的跑批正在用的节点，实测曾毁掉一整轮建图。
LOCK=/tmp/ros_batch.lock
exec 9>"$LOCK"
flock -n 9 || { echo "拒绝启动：$LOCK 已被另一个 ROS 跑批持有（禁止并行）"; exit 1; }
# 第二道闸：场上若有不持锁的跑批（如 /tmp/run_ntu_degen.sh）在跑，直接拒绝启动。
source /home/ywj/git/Super-LIVO/tools/ros_batch_guard.sh

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
source /home/ywj/git/Super-LIVO/install/setup.bash

pkill -x super_lio_node 2>/dev/null; pkill -x visual_a_node 2>/dev/null
pkill -x visual_b_node 2>/dev/null; pkill -f 'fastlivo_mapping.*NTU_VIRAL.yaml' 2>/dev/null
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
  ros2 bag play "$BAG" --rate "$RATE" > /tmp/bagplay_${SEQ}_${SIDE}.log 2>&1
  sleep 5
  kill -INT $REC 2>/dev/null; sleep 4; kill $REC 2>/dev/null
  kill -INT $NODE_PID 2>/dev/null; sleep 8
  kill $BB 2>/dev/null
  pkill -f 'fastlivo_mapping.*NTU_VIRAL.yaml' 2>/dev/null
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

ros2 run super_lio super_lio_node --ros-args --params-file "$SL_NTU" \
  -p lio.observe.degeneracy_detect_en:=true -p lio.observe.degen_trans_only:=1 \
  > /tmp/slio_${SEQ}_${SIDE}.log 2>&1 &
# 可选：用环境变量覆盖门控阈值（做阈值敏感性/链路验证时用），不设则用 params 文件
GATE_ARGS=()
[ -n "$GATE_LO" ] && GATE_ARGS+=(-p gate.cond_lo:=$GATE_LO)
[ -n "$GATE_HI" ] && GATE_ARGS+=(-p gate.cond_hi:=$GATE_HI)
ros2 run "$NODE_PKG" "$NODE_EXE" --ros-args --params-file "$PARAMS" "${GATE_ARGS[@]}" \
  > "$VLOG" 2>&1 &
sleep 6
ros2 bag record -o "$ODOMDIR" /lio/odom /lio/degeneracy /lio/cloud_body_dense "$OWN" \
  > /tmp/rec_${SEQ}_${SIDE}.log 2>&1 &
REC=$!
sleep 2
ros2 bag play "$BAG" --rate "$RATE" > /tmp/bagplay_${SEQ}_${SIDE}.log 2>&1
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
