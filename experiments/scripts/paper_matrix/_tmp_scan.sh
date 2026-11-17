#!/usr/bin/env bash
# 临时：扫描各臂 latency.csv 的 algo 分布（用完删除）
for f in $(find /home/ywj/slam_data/paper_20261117 -name latency.csv | sort); do
  echo "### ${f#/home/ywj/slam_data/paper_20261117/}"
  python3 /home/ywj/git/Super-LIVO/experiments/scripts/paper_matrix/_tmp_inspect2.py "$f" | sed -n '2,4p'
done
