#!/bin/bash
echo "=== /tmp/run_b2.sh ==="
cat /tmp/run_b2.sh 2>/dev/null
echo "=== /tmp/b3_smoke.sh ==="
cat /tmp/b3_smoke.sh 2>/dev/null
echo "=== /tmp/b3_build.sh ==="
cat /tmp/b3_build.sh 2>/dev/null
echo "=== odom run dirs ==="
ls -latd /home/ywj/slam_data/*odom* 2>/dev/null | head -20
ls -latd /home/ywj/*run* /home/ywj/slam_data/room_slam/* 2>/dev/null | head -20
echo "=== FD edit present? ==="
grep -n 'VisB3-FD' /home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/src/visual/visual_eskf_inject.cpp
echo "=== git status ==="
cd /home/ywj/git/Super-LIVO/third_party/Super-LIO && git status --short | head -30
