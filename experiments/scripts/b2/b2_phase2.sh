#!/usr/bin/env bash
# B2 验证流水线 第二阶段：全量 160 s + 评测
CFGDIR=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config
B2=$CFGDIR/livox_360_visa_room_b2.yaml
OUT=/home/ywj/slam_data/room_slam

echo "########## FULL RUN b2 ##########"
date +'start %F_%T'
bash /tmp/run_b2.sh room_b2 "$B2" 0
date +'end %F_%T'
echo "--- injected 总行数: $(grep -cE '\[VisRot\] inl=' /tmp/slio_room_b2.log)"
grep -E '\[VisRot\] inl=' /tmp/slio_room_b2.log | tail -5
echo "--- none 状态行:"
grep -E '\[VisRot\] none' /tmp/slio_room_b2.log | tail -4
echo "--- 警告/错误:"
grep -iE 'error|what\(\)|Segmentation|nan|Exception|FATAL' /tmp/slio_room_b2.log | head -10

echo
echo "########## EVAL ##########"
python3 /tmp/eval_loop.py "$OUT/run_room_b2" "b2"
echo
python3 /tmp/eval_loop.py "$OUT/run_room_off" "off-baseline"
echo "=== PHASE2 END ==="
