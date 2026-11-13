#!/usr/bin/env bash
# RTK-SLAM construction_seq2 消融的统一评测入口。
#   1) 跑 run：      BEAM=0.003 TAG=beam3 OUT=.../run_lio_beam3 bash tools/run_rtk.sh
#   2) 评 run：      bash tools/rtk_report.sh beam3:super_lio_beam3
#      （pair = <run 目录后缀>:<方法名>，run 目录固定为 slam_data/rtk_slam/run_lio_<后缀>；
#        不带参数 = 评下面 ALL 里登记的全部 run）
# 导出一律经 rtk_traj_prep.py（内部按 odom header 时间戳排序，见 §19.12）。
set -e
cd /home/ywj/git/Super-LIVO

RTK=/home/ywj/slam_data/rtk_slam
SRC=$RTK/construction_seq2_r2
ALL=("seq2:super_lio" "beam3:super_lio_beam3" "beam:super_lio_beam")
PAIRS=("$@")
[ ${#PAIRS[@]} -eq 0 ] && PAIRS=("${ALL[@]}")

NAMES=()
for pair in "${PAIRS[@]}"; do
  tag=${pair%%:*}; m=${pair#*:}
  echo "### $tag -> $m"
  python3 tools/rtk_traj_prep.py --odom-bag "$RTK/run_lio_$tag" --src-bag "$SRC" \
    --method "$m" --seq construction_seq2 | grep -E 'odom poses|排序|航向'
  NAMES+=("$m")
done

echo
echo "########## 绝对 ATE（未对齐，官方协议）##########"
(cd /home/ywj/slam_data/rtk_slam/eval_ref && python3 eval/eval.py --seq construction_seq2)
echo
echo "########## 分检查点 / 分环境 ##########"
python3 tools/rtk_per_checkpoint.py "${NAMES[@]}" fast_lio_sam
echo
echo "########## 逐帧抖动 + 路径 ##########"
python3 tools/rtk_jitter.py "${NAMES[@]}"
echo
echo "########## 瞬移事件 ##########"
python3 tools/rtk_events.py "${NAMES[@]}"
echo
echo "########## 刚体偏置分解 ##########"
python3 tools/rtk_bias.py "${NAMES[@]}" fast_lio_sam
echo
echo "########## 累积漂移判据 ##########"
python3 tools/rtk_drift.py "${NAMES[0]}" 2>&1 | sed -n '/回归/,$p'
