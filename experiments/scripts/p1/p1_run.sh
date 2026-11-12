#!/bin/bash
# 方案①单次 room 跑批 + 指标抽取（紧凑输出）
# 用法: bash /tmp/p1_run.sh <tag> <mode> [参数...]
#   mode = off | b2 | b3 | both
set +u
Y=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/livox_360_visa_room_b3.yaml
TAG=$1
MODE=$2
shift 2 2>/dev/null
BASE=""
case "$MODE" in
  off)  BASE="-p lio.visual.enable:=false" ;;
  b2)   BASE="-p lio.visual.b2.enable:=true -p lio.visual.b3.enable:=false" ;;
  b3)   BASE="-p lio.visual.b2.enable:=false -p lio.visual.b3.enable:=true" ;;
  both) BASE="-p lio.visual.b2.enable:=true -p lio.visual.b3.enable:=true" ;;
esac

bash /tmp/b3_one.sh "$TAG" 0 $BASE "$@" > /tmp/p1_raw_$TAG.txt 2>&1
L=/tmp/slio_$TAG.log

echo "-- run $TAG mode=$MODE extra: $*"
grep -a '\[Param\] visual/b2' "$L" | head -1
python3 /tmp/eval_loop.py /home/ywj/slam_data/room_slam/run_$TAG "$TAG" > /tmp/p1_eval_$TAG.txt 2>&1
sed -n '2,4p' /tmp/p1_eval_$TAG.txt
echo "REJ_LINES=$(grep -ac 'VisB2. reject chi2' "$L")"
echo "GATE: $(grep -a '\[VisB2gate\]' "$L" | tail -1)"
echo "LAST: $(grep -a '\[VisRot\] inl=' "$L" | tail -1)"
echo "B3: $(grep -a '\[VisB3\] t=' "$L" | tail -1)"
python3 /tmp/p1_safe.py /home/ywj/slam_data/room_slam/run_$TAG "$TAG" 2>&1
grep -a 'POST-CLEAN' /tmp/p1_raw_$TAG.txt
echo "P1RUN-DONE $TAG"
