#!/usr/bin/env bash
OUT=/home/ywj/slam_data/room_slam
echo "################ off vs s0001 逐段差异（stamp 对齐）"
python3 /tmp/b2_diag.py "$OUT/run_room_off" "$OUT/run_room_b2_s0001" off b2_s0001 2>&1 | tail -12
echo "################ off vs s002 逐段差异"
python3 /tmp/b2_diag.py "$OUT/run_room_off" "$OUT/run_room_b2_s002" off b2_s002 2>&1 | tail -8
echo "################ 追加 σ=0.0005 / 0.001"
bash /tmp/b2_one.sh b2_s0005 0.0005 2>&1 | grep -vE '^\s+[0-9]+\.[0-9]+\s+-'
bash /tmp/b2_one.sh b2_s001 0.001 2>&1 | grep -vE '^\s+[0-9]+\.[0-9]+\s+-'
