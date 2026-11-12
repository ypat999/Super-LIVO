#!/usr/bin/env bash
# 决定性实验第三批：把 sigma_rot 压到 1e-4，让 S≈HPHᵀ（门真正由 P 驱动），
# 检验"高 P → 接受率升高"的自适应机理是否存在（§7.3 机理直接验证）。
LOG=/tmp/deci_batch3.log
: > "$LOG"
bash /tmp/deci_one.sh dec_b_d_s4 b2 1e-4 0.1 1 1e-4 >> "$LOG" 2>&1
bash /tmp/deci_one.sh dec_b_a_s4 b2 3e-2 1.0 1 1e-4 >> "$LOG" 2>&1
echo "BATCH3-ALL-DONE" >> "$LOG"
