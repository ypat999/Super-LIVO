#!/usr/bin/env bash
# 对一次 run 产出 ENU 轨迹并出绝对 ATE。
#   用法: bash tools/rtk_eval.sh <odom_bag_dir> <method_name>
set -e
BAG=${1:?需要 odom bag 目录}
M=${2:?需要方法名}
RTK=/home/ywj/slam_data/rtk_slam
python3 /home/ywj/git/Super-LIVO/tools/rtk_traj_prep.py \
  --odom-bag "$BAG" --src-bag $RTK/construction_seq2_r2 \
  --method "$M" --seq construction_seq2
echo
cd $RTK/eval_ref
python3 eval/eval.py --method "$M" --seq construction_seq2
