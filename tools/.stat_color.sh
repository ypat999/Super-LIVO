#!/usr/bin/env bash
# 统计上色帧分布
grep -a -oE '[0-9]+/[0-9]+ 点上色' /tmp/vis_med.log | awk '{split($1,a,"/"); print a[1]}' | sort -n |
  awk '{v[NR]=$1; s+=$1} END {
    printf "n=%d 均值=%.0f min=%d p25=%d 中位=%d p75=%d max=%d\n",
      NR, s/NR, v[1], v[int(NR*0.25)], v[int(NR/2)], v[int(NR*0.75)], v[NR]}'
