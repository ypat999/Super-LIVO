#!/usr/bin/env bash
# 临时：查看各臂 pcd 产物（用完删除）
ROOT=/home/ywj/slam_data/paper_20261117
for d in ntu01/sl ntu01/a ntu01/b ntu01/fl2 ntu03/sl retail/a move/b move/fl2; do
  echo "== $d"
  find "$ROOT/$d/pcd" -type f -name '*.pcd' -printf '%10s  %p\n' 2>/dev/null | head -15
done
