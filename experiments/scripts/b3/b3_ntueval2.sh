#!/bin/bash
GT=/home/ywj/slam_data/ntu_viral/gt/eee_03/ground_truth.csv
BASE=/home/ywj/slam_data/ntu_viral/run_ntu
echo "########## ATE 全七档"
python3 /home/ywj/git/Super-LIVO/tools/odom_vs_gt.py --gt "$GT" \
  --bag "$BASE"_off --bag "$BASE"_b2 --bag "$BASE"_bd05 --bag "$BASE"_bd10 --bag "$BASE"_bd20 \
  --bag "$BASE"_bd10s002 --bag "$BASE"_bd10s005 \
  --topic /lio/odom --label off --label B2 --label bd05s005 --label bd10s005 --label bd20s005 \
  --label bd10s002 --label bd10s0005 2>&1 | tail -30
echo "########## 终点口径"
for t in off bd10s002 bd10s005; do
  python3 /tmp/eval_loop.py "$BASE"_$t ntu_$t 2>&1 | sed -n '2,5p'
done
echo "########## 新两档诊断"
for t in bd10s002 bd10s005; do
  L=/tmp/slio_ntu_$t.log
  echo "----- $t"
  grep -a '\[VisB3\] t=' "$L" | tail -1
  echo "reject=$(grep -ac 'reject chi2' "$L")"
  grep -a '\[VisB3bg\]' "$L" | awk 'NR%25==1{printf "%s ", $5}'
  echo
  grep -a '\[VisB3bg\]' "$L" | tail -1
  grep -aiE 'nan|Segmentation|FATAL' "$L" | head -3
done
echo "EVAL2-DONE"
