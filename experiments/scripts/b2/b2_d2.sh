#!/usr/bin/env bash
OUT=/home/ywj/slam_data/room_slam
echo "############ off vs s0001 (σ=1e-4)"
python3 /tmp/b2_diag.py "$OUT/run_room_off" "$OUT/run_b2_s0001" off s0001 2>&1 | tail -14
echo
echo "############ off vs s005 (σ=5e-3, 宽松门控)"
python3 /tmp/b2_diag.py "$OUT/run_room_off" "$OUT/run_b2_s005" off s005 2>&1 | tail -10
