#!/usr/bin/env bash
# 论文对比矩阵驱动：按「先短后长」顺序串行跑 6 序列 × 4 臂。
# 已完成（<OUTROOT>/<seq>/<arm>/done）自动跳过，可断点续跑。
#
#   wsl bash experiments/scripts/paper_matrix/run_matrix.sh
#   ARMS="sl a b fl2" SEQS="move retail" bash ...   # 只跑子集
#   SMOKE=25 bash ...                              # 冒烟（不写 done）
set -o pipefail
REPO=/home/ywj/git/Super-LIVO
SEQS=${SEQS:-"move retail room ntu03 ntu01 rtk"}
ARMS=${ARMS:-"sl a b fl2"}
RATE=${RATE:-1.0}
SMOKE=${SMOKE:-0}
export RATE SMOKE
OUTROOT=${OUTROOT:-/home/ywj/slam_data/paper_20261117}
export OUTROOT
LOG=$OUTROOT/matrix.log
mkdir -p "$OUTROOT"

for seq in $SEQS; do
  for arm in $ARMS; do
    echo "=========== $(date -Is) $seq/$arm ===========" | tee -a "$LOG"
    bash "$REPO/experiments/scripts/paper_matrix/run_one.sh" "$seq" "$arm" 2>&1 | tee -a "$LOG"
    rc=${PIPESTATUS[0]}
    [ "$rc" = "0" ] || echo "!! $seq/$arm 退出码 $rc" | tee -a "$LOG"
    sleep 5
  done
done
echo "=========== $(date -Is) MATRIX DONE ===========" | tee -a "$LOG"
