#!/usr/bin/env bash
# 等 retail A 跑完
for i in $(seq 1 40); do
  pgrep -f 'run_ab_retai[l].sh a' > /dev/null || break
  sleep 5
done
cat /home/ywj/slam_data/output/retail_ab_1114/a/meta.txt 2>/dev/null
