#!/usr/bin/env bash
# 等 retail B 跑完
for i in $(seq 1 40); do
  pgrep -f 'run_ab_retai[l].sh b' > /dev/null || break
  sleep 5
done
cat /home/ywj/slam_data/output/retail_ab_1114/b/meta.txt 2>/dev/null
echo "=== E-2 进度"
cd /home/ywj/git/Super-LIVO/third_party/Super-LIO/exp/results
for d in m40_e2_1f_r25 m40_e2_1f_r15 m40_e2_ns_r25 m40_e2_ns_r15; do
  printf '%s ' "$d"
  if [ -f "$d/meta.txt" ]; then grep -hE 'odom_rows' "$d/meta.txt" | tr '\n' ' '; echo; else echo pending; fi
done
