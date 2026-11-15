#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
cd /home/ywj/git/Super-LIVO
colcon build --packages-select super_livo_visual_a super_livo_visual_b \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -30
