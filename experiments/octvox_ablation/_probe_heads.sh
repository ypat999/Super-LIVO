#!/usr/bin/env bash
for f in \
  /home/ywj/slam_data/official_vio_20261114/ladder/R0.pcd \
  /home/ywj/slam_data/official_vio_20261114/ladder/R5.pcd \
  /home/ywj/slam_data/colored_fl2_20261114/fl2_A/super_livo_visual_a_colored.pcd \
  /home/ywj/slam_data/colored_fl2_20261114/fl2_B/super_livo_visual_b_colored.pcd ; do
  echo "=== $f"
  ls -l "$f" 2>/dev/null | awk '{print "    bytes:", $5}'
  head -c 400 "$f" 2>/dev/null | tr -d '\0' | sed -n '1,12p'
  echo
done
