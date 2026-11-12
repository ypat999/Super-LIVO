#!/usr/bin/env bash
for d in off on force force2 latch; do
  python3 /tmp/eval_loop.py "/home/ywj/slam_data/room_slam/run_room_${d}" "${d}" 2>&1 | head -5
done
