#!/bin/bash
# 单次 room 跑批（前台、持锁、前后清场、跑完立刻评估）
# 用法: bash /tmp/b3_one.sh <tag> <SMOKE秒|0> [额外 -p 参数...]
set +u
Y=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/livox_360_visa_room_b3.yaml
TAG=$1
SMOKE=${2:-0}
shift 2 2>/dev/null
EXTRA="$*"

exec 9>/tmp/ros_batch.lock
flock 9

pre_clean() {
  for p in $(pgrep -f 'super_lio_node --ros-args'); do kill -9 "$p" 2>/dev/null; done
  for p in $(pgrep -f 'ros2 bag (record|play)'); do kill -9 "$p" 2>/dev/null; done
  sleep 2
}
pre_clean
echo "### PRE-CLEAN remain: [$(pgrep -af 'super_lio_node|ros2 bag' | grep -av pgrep | tr '\n' ';')]"

bash /tmp/run_b2.sh "$TAG" "$Y" "$SMOKE" $EXTRA

for p in $(pgrep -f 'super_lio_node --ros-args'); do kill -9 "$p" 2>/dev/null; done
for p in $(pgrep -f 'ros2 bag (record|play)'); do kill -9 "$p" 2>/dev/null; done
sleep 2
echo "### POST-CLEAN remain: [$(pgrep -af 'super_lio_node|ros2 bag' | grep -av pgrep | tr '\n' ';')]"

echo "### EVAL $TAG"
python3 /tmp/eval_loop.py "/home/ywj/slam_data/room_slam/run_$TAG" "$TAG" 2>&1 | tail -12
echo "### VisB3 tail"
grep -a '\[VisB3\] t=' "/tmp/slio_$TAG.log" 2>/dev/null | tail -1
echo "### VisB3bg 首尾"
grep -a '\[VisB3bg\]' "/tmp/slio_$TAG.log" 2>/dev/null | head -2
grep -a '\[VisB3bg\]' "/tmp/slio_$TAG.log" 2>/dev/null | tail -2
echo "### bgz 序列(每20条取1)"
grep -a '\[VisB3bg\]' "/tmp/slio_$TAG.log" 2>/dev/null | awk 'NR%20==1{print $3,$4,$5}'
echo "### reject 统计"
grep -ac 'reject chi2' "/tmp/slio_$TAG.log" 2>/dev/null
echo "ONE-DONE $TAG"
