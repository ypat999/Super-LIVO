#!/usr/bin/env bash
# 临时：补跑/重跑（用完删除）。
# 依据 2026-11-17 上午的复盘：
#   1) 探针雷达 QoS 已改 BEST_EFFORT/depth=20 → move×4、retail b/sl、room/sl 需重测；
#   2) FL2 相机内参远程读取是 100 ms 发现竞态 → ntu03/fl2、ntu01/fl2、retail/fl2 需重跑；
#   3) 上轮 rtk 四臂因孤儿进程占锁全被跳过 → 需补跑。
set -o pipefail
REPO=/home/ywj/git/Super-LIVO
LOG=/home/ywj/slam_data/paper_20261117/rerun.log
export FORCE=1
cd "$REPO" || exit 1

run() {
  echo "### $(date -Is) rerun seq=$1 arms=$2 ###" | tee -a "$LOG"
  SEQS="$1" ARMS="$2" bash experiments/scripts/paper_matrix/run_matrix.sh
}

run ntu03  "fl2"          # 先验证黑板竞态修复（最短的 FL2 长序列）
run move   "sl a b fl2"
run retail "b sl fl2"
run room   "sl"
run ntu01  "fl2"
run rtk    "sl a b fl2"
echo "### $(date -Is) RERUN DONE ###" | tee -a "$LOG"
