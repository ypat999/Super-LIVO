#!/bin/bash
# 从既有日志+odom库补齐一张表：每个 tag 一行块。用法: bash /tmp/p1_tab.sh tag1 tag2 ...
for TAG in "$@"; do
  L=/tmp/slio_$TAG.log
  D=/home/ywj/slam_data/room_slam/run_$TAG
  echo "=====TAG $TAG"
  if [ -f "$L" ]; then
    grep -a '[Param] visual/b2' "$L" | head -1
    grep -a '[VisB2gate]' "$L" | tail -1
    grep -a '[VisRot] inl=' "$L" | tail -1
    grep -a '[VisB3] t=' "$L" | tail -1
    echo "REJ=$(grep -ac 'VisB2. reject chi2' "$L")"
  else
    echo "NOLOG"
  fi
  if [ -d "$D" ]; then
    python3 /tmp/eval_loop.py "$D" "$TAG" 2>&1 | sed -n '2,4p'
    python3 /tmp/p1_safe.py "$D" "$TAG" 2>&1
  else
    echo "NORUNDIR"
  fi
done
echo "TAB-DONE"
