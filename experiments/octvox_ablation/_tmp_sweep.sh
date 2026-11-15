#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
cd /home/ywj/slam_data/colored_superlio_official_20261114
for v in "official 0.1" "lidar 0.1" "official 0.0" "official 0.05" "official 0.15"; do
  set -- $v
  echo "=== ext=$1 offset=$2"
  python3 /home/ywj/git/Super-LIVO/experiments/octvox_ablation/15_superlio_official_color.py \
    --out "_sweep_$1_$2.pcd" --ext "$1" --offset "$2" --max-frames 300 --quick-metric 2>&1 \
    | grep -E "ext=|输入点|quick"
done
