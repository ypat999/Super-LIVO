#!/bin/bash
# NTU 五档评测：官方 ATE + 闭环/终点 yaw 口径
GT=/home/ywj/slam_data/ntu_viral/gt/eee_03/ground_truth.csv
BASE=/home/ywj/slam_data/ntu_viral/run_ntu
echo "########## 1) 官方协议 ATE (odom_vs_gt.py)"
python3 /home/ywj/git/Super-LIVO/tools/odom_vs_gt.py --gt "$GT" \
  --bag "$BASE"_off --bag "$BASE"_b2 --bag "$BASE"_bd05 --bag "$BASE"_bd10 --bag "$BASE"_bd20 \
  --topic /lio/odom --label off --label B2 --label bd05 --label bd10 --label bd20 \
  --out /tmp/ntu_b3_ate.png 2>&1 | tail -60
echo
echo "########## 2) 闭环/终点口径 (eval_loop.py)"
for t in off b2 bd05 bd10 bd20; do
  python3 /tmp/eval_loop.py "$BASE"_$t ntu_$t 2>&1 | head -6
done
echo "EVAL-DONE"
