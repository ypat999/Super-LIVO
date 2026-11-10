#!/usr/bin/env bash
# 拉取 rtk-slam-eval 仓库里的小体量参考件（GT 控制点、标定、RTK 输入、
# FAST-LIO-SAM 基线轨迹、评测脚本），供 Super-LIVO 直接对照使用。
D=/home/ywj/slam_data/rtk_slam/eval_ref
B=https://raw.githubusercontent.com/Willyzw/rtk-slam-eval/main
mkdir -p "$D"/{ground_truth,calib,data,trajectories/fast_lio_sam,eval} || exit 1

get() {  # $1=仓库内路径 $2=本地相对路径
  curl -sfL --max-time 90 -o "$D/$2" "$B/$1" && echo "OK  $2" || echo "FAIL $1"
}

get ground_truth/construction_seq2.csv        ground_truth/construction_seq2.csv
get ground_truth/construction_seq1.csv        ground_truth/construction_seq1.csv
get ground_truth/stadtgarten_seq1.csv         ground_truth/stadtgarten_seq1.csv
get ground_truth/stadtgarten_seq2.csv         ground_truth/stadtgarten_seq2.csv
get calib/calib.yaml                          calib/calib.yaml
get data/construction_seq2/rtk.txt            data/construction_seq2_rtk.txt
get trajectories/fast_lio_sam/construction_seq2/traj_online.txt   trajectories/fast_lio_sam/construction_seq2_online.txt
get trajectories/fast_lio_sam/construction_seq2/traj_offline.txt  trajectories/fast_lio_sam/construction_seq2_offline.txt
get trajectories/fast_lio_sam/construction_seq2/enu_origin.json   trajectories/fast_lio_sam/construction_seq2_enu_origin.json
for f in eval.py metrics.py readers.py coords.py transform_imu_to_base.py plot.py; do
  get eval/$f eval/$f
done
get README.md README.md
get requirements.txt requirements.txt

echo "=== 文件清单 ==="
find "$D" -type f | sort | xargs wc -l | tail -25
echo "=== GT 前 5 行 ==="
head -5 "$D/ground_truth/construction_seq2.csv"
echo "=== ENU origin ==="
cat "$D/trajectories/fast_lio_sam/construction_seq2_enu_origin.json"
echo
echo "=== calib.yaml ==="
cat "$D/calib/calib.yaml"
