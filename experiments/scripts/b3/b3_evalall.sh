#!/bin/bash
# 对已存在的 run 目录重新评估，输出对齐表：tag | loop_pos | loop_yaw | end_pos | end_yaw
printf '%-10s %9s %9s %9s %9s\n' TAG loop_pos loop_yaw end_pos end_yaw
for t in "$@"; do
  d=/home/ywj/slam_data/room_slam/run_$t
  if [ ! -d "$d" ]; then printf '%-10s %s\n' "$t" MISSING; continue; fi
  python3 /tmp/eval_loop.py "$d" "$t" > /tmp/eval_$t.txt 2>&1
  lp=$(grep -aoP '位置偏差 \K[-0-9.]+' /tmp/eval_$t.txt | head -1)
  ly=$(grep -aoP 'yaw 偏差 \K[-0-9.]+' /tmp/eval_$t.txt | head -1)
  ep=$(grep -aoP '位置偏差 \K[-0-9.]+' /tmp/eval_$t.txt | sed -n 2p)
  ey=$(grep -aoP 'yaw 偏差 \K[-0-9.]+' /tmp/eval_$t.txt | sed -n 2p)
  fr=$(grep -aoP '帧数 \K[0-9]+' /tmp/eval_$t.txt)
  printf '%-10s %9s %9s %9s %9s  frames=%s\n' "$t" "$lp" "$ly" "$ep" "$ey" "$fr"
done
