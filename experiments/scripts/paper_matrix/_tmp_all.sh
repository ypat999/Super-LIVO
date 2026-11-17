#!/usr/bin/env bash
# 临时：全臂汇总（用完删除）。label 用 <seq>/<arm>，避免跨序列重名。
set -o pipefail
REPO=/home/ywj/git/Super-LIVO
ROOT=/home/ywj/slam_data/paper_20261117
LABELS=()
PATHS=()
for f in $(find "$ROOT" -name latency.csv | sort); do
  d=$(dirname "$f")
  LABELS+=(--label "$(basename "$(dirname "$d")")/$(basename "$d")")
  PATHS+=("$f")
done
python3 "$REPO/tools/latency_summary.py" "${LABELS[@]}" "$@" "${PATHS[@]}" 2>&1
