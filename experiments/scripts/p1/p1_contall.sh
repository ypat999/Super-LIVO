#!/bin/bash
# 对所有既有 room run 目录补 CONT 连续性行；可选：传 tag 列表只跑指定几个
cd /home/ywj/slam_data/room_slam || exit 1
if [ "$#" -gt 0 ]; then
  DIRS=""
  for t in "$@"; do DIRS="$DIRS run_$t"; done
else
  DIRS=$(ls -d run_*)
fi
for d in $DIRS; do
  ls "$d"/*.db3 >/dev/null 2>&1 || continue
  python3 /tmp/p1_cont.py "$d" "${d#run_}" 2>&1
done
echo CONTALL-DONE
