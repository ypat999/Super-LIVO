#!/bin/bash
# 汇总评估：对若干 tag 打印回环/终点指标
for t in "$@"; do
  d=/home/ywj/slam_data/room_slam/run_$t
  [ -d "$d" ] || { echo "-- $t: NO DIR"; continue; }
  echo "== $t =="
  python3 /tmp/eval_loop.py "$d" "$t" 2>&1 | head -8
  python3 /tmp/eval_loop.py "$d" "$t" 2>&1 | tail -3
done
