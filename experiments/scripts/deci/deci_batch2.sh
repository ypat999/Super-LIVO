#!/usr/bin/env bash
# 决定性实验第二批：同 build 的 off 基线 + 高噪声两组（b2_d 已单独跑完）
LOG=/tmp/deci_batch2.log
: > "$LOG"
bash /tmp/deci_one.sh dec_off_d off 1e-4 0.1 0 >> "$LOG" 2>&1
bash /tmp/deci_one.sh dec_off_a off 3e-2 1.0 0 >> "$LOG" 2>&1
bash /tmp/deci_one.sh dec_b_a  b2  3e-2 1.0 1 >> "$LOG" 2>&1
echo "BATCH2-ALL-DONE" >> "$LOG"
