#!/bin/bash
# NTU eee_03 单跑：bash /tmp/b3_ntu.sh <tag> <SMOKE秒|0> [额外 -p 参数...]
# 0.5 倍速 => 181.4s bag -> ~363s 墙钟。持 /tmp/ros_batch.lock，跑前跑后清场。
set +u
CFG=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/NTU_visa_b3.yaml
BAG=/home/ywj/slam_data/ntu_viral/bags2/eee_03
TAG=$1
SMOKE=${2:-0}
shift 2 2>/dev/null
EXTRA="$*"
OUT=/home/ywj/slam_data/ntu_viral/run_ntu_$TAG
log=/tmp/slio_ntu_$TAG.log

exec 9>/tmp/ros_batch.lock
flock 9

# 先看场：有别人的节点在跑就退出，绝不误杀（清场只在确认无残留后做）
LEFT=$(pgrep -af 'super_lio_node --ros-args|ros2 bag (record|play)' | grep -av pgrep | tr '\n' ';')
echo "### PRE remain: [$LEFT]"
[ -n "$LEFT" ] && { echo "ABORT: 有残留，拒绝并行"; exit 1; }
for p in $(pgrep -f 'super_lio_node --ros-args'); do kill -9 "$p" 2>/dev/null; done
for p in $(pgrep -f 'ros2 bag (record|play)'); do kill -9 "$p" 2>/dev/null; done
sleep 2

source /opt/ros/humble/setup.bash
source /home/ywj/git/3d_dog_navi_ros2/install/setup.bash 2>/dev/null
source /home/ywj/git/Super-LIVO/third_party/Super-LIO/install/setup.bash
export ROS_DOMAIN_ID=71
export ROS_LOCALHOST_ONLY=1
rm -rf "$OUT"

# 与 tools/run_ntu.sh 的 NTU 协议保持一致：开退化检测 + 平移单方向约束
ros2 run super_lio super_lio_node --ros-args --params-file "$CFG" \
  -p lio.observe.degeneracy_detect_en:=true -p lio.observe.degen_trans_only:=1 \
  -p lio.map.save_map:=false $EXTRA > "$log" 2>&1 &
SL=$!
sleep 8
ros2 bag record -o "$OUT" /lio/odom > /tmp/rec_ntu_$TAG.log 2>&1 &
REC=$!
sleep 2
if [ "$SMOKE" != "0" ]; then
  timeout -s INT "$SMOKE" ros2 bag play "$BAG" --rate 0.5 > /tmp/play_ntu_$TAG.log 2>&1
else
  ros2 bag play "$BAG" --rate 0.5 > /tmp/play_ntu_$TAG.log 2>&1
fi
sleep 3
kill -INT $REC 2>/dev/null; sleep 3; kill -9 $REC 2>/dev/null
pkill -INT -P $SL 2>/dev/null; kill -INT $SL 2>/dev/null; sleep 6
pkill -9 -P $SL 2>/dev/null; kill -9 $SL 2>/dev/null
for p in $(pgrep -f 'super_lio_node --ros-args'); do kill -9 "$p" 2>/dev/null; done
sleep 2
echo "### POST remain: [$(pgrep -af 'super_lio_node --ros-args|ros2 bag (record|play)' | grep -av pgrep | tr '\n' ';')]"
ros2 bag info "$OUT" 2>/dev/null | grep -aE 'Messages|Duration'
echo "### log lines=$(wc -l < $log)"
grep -a 'Param] visual/b3' "$log" | tail -1
echo "NTU-DONE $TAG"
