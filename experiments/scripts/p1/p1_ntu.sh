#!/bin/bash
# NTU eee_03 单次 + ATE 评估。用法: bash /tmp/p1_ntu.sh <tag> [额外 -p ...]
# tag=off 时自动加 -p lio.visual.enable:=false
set +u
TAG=$1
shift
EXTRA="$*"
case "$TAG" in
  off) EXTRA="-p lio.visual.enable:=false" ;;
  b2only) EXTRA="$EXTRA -p lio.visual.b2.enable:=true -p lio.visual.b3.enable:=false" ;;
esac
bash /tmp/b3_ntu.sh "$TAG" 0 $EXTRA > /tmp/p1_ntu_raw_$TAG.txt 2>&1
L=/tmp/slio_ntu_$TAG.log
O=/home/ywj/slam_data/ntu_viral/run_ntu_$TAG
GT=/home/ywj/slam_data/ntu_viral/gt/eee_03/ground_truth.csv
echo "--NTU $TAG extra: $EXTRA"
grep -a 'PRE remain\|ABORT' /tmp/p1_ntu_raw_$TAG.txt | head -2
grep -a '[Param] visual/b2' "$L" | head -1
tail -2 /tmp/p1_ntu_raw_$TAG.txt
echo "REJ=$(grep -ac 'VisB2. reject chi2' "$L")"
grep -a '[VisB2gate]' "$L" | tail -1
grep -a '[VisRot] inl=' "$L" | tail -1
grep -a '[VisB3] t=' "$L" | tail -1
python3 /home/ywj/git/Super-LIVO/tools/odom_vs_gt.py --gt "$GT" --bag "$O" --topic /lio/odom --label "$TAG" 2>&1 | sed -n '1,6p'
python3 /tmp/p1_safe.py "$O" ntu_$TAG 2>&1
echo "P1NTU-DONE $TAG"
