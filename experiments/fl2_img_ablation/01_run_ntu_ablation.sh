#!/usr/bin/env bash
# NTU eee_03 官方 FAST-LIVO2「有图像 / 去除图像」成对消融（有真值，判决性实验）。
# 3 轮 × 2 条件，交错执行（m1r1, m0r1, m1r2, ...），避免机器状态漂移系统性偏袒某一条件。
#
#   bash experiments/fl2_img_ablation/01_run_ntu_ablation.sh
#
# 产物：OUT/ntu/m{img_en}r{rep}_{pose.txt,node.log,odom/}
set -o pipefail

SEQ=${SEQ:-eee_03}
REPS=${REPS:-3}
OUT=/home/ywj/slam_data/fl2_img_abl_20261110
FLLOG=/home/ywj/git/Super-LIVO/third_party/FAST-LIVO2-ROS2/Log
cd /home/ywj/git/Super-LIVO || exit 1
mkdir -p "$OUT/ntu"

for r in $(seq 1 "$REPS"); do
  for m in 1 0; do
    tag="m${m}r${r}"
    echo "===== [$(date +%H:%M:%S)] NTU $SEQ img_en=$m rep=$r ====="
    rm -f "$FLLOG/result/${SEQ}_$tag.txt"
    IMG_EN=$m FL_TAG="_$tag" SEQ=$SEQ bash tools/run_ntu.sh fl 2>&1 | tail -4
    pose="$FLLOG/result/${SEQ}_$tag.txt"
    if [ -f "$pose" ]; then
      mv "$pose" "$OUT/ntu/${tag}_pose.txt"
      cp "/tmp/fl_${SEQ}_fl.log" "$OUT/ntu/${tag}_node.log"
      mv /home/ywj/slam_data/ntu_odom_${SEQ}_fl "$OUT/ntu/${tag}_odom" 2>/dev/null
      echo "[$tag] pose $(wc -l < "$OUT/ntu/${tag}_pose.txt") 行"
      grep -acE 'dummy|Dummy' "$OUT/ntu/${tag}_node.log" || true
    else
      echo "[$tag] !! 未找到 $pose"; tail -30 "/tmp/fl_${SEQ}_fl.log"
    fi
    sleep 5
  done
done
echo "ALL NTU DONE"
