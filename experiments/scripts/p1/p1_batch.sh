#!/bin/bash
# 批量跑 room B2 网格。用法: bash /tmp/p1_batch.sh "tag,sigma,thresh" ...
for spec in "$@"; do
  TAG=$(echo "$spec" | awk -F, '{print $1}')
  S=$(echo "$spec"   | awk -F, '{print $2}')
  T=$(echo "$spec"   | awk -F, '{print $3}')
  bash /tmp/p1_run.sh "$TAG" b2 -p lio.visual.sigma_rot:=$S -p lio.visual.chi2_thresh:=$T
done
echo "BATCH-DONE"
