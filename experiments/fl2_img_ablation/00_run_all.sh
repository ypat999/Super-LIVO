#!/usr/bin/env bash
# 串行跑完整消融：先 NTU（有真值，判决性），再 Retail（补齐 odom+PCD 成对资产）。
# 两者必须串行，不能并发 —— 并发会抢 CPU 造成处理延迟，污染厘米级对照。
#
#   REPS=3 bash experiments/fl2_img_ablation/00_run_all.sh
set -o pipefail
cd /home/ywj/git/Super-LIVO || exit 1
REPS=${REPS:-3}
LOG=/home/ywj/slam_data/fl2_img_abl_20261110/run_all.log
mkdir -p "$(dirname "$LOG")"

echo "##### [$(date +%F' '%H:%M:%S)] NTU eee_03 开始 #####" | tee -a "$LOG"
REPS=$REPS bash experiments/fl2_img_ablation/01_run_ntu_ablation.sh 2>&1 | tee -a "$LOG"
echo "##### [$(date +%F' '%H:%M:%S)] Retail_Street 开始 #####" | tee -a "$LOG"
REPS=$REPS bash experiments/fl2_img_ablation/03_run_retail_ablation.sh 2>&1 | tee -a "$LOG"
echo "##### [$(date +%F' '%H:%M:%S)] 全部结束 #####" | tee -a "$LOG"
