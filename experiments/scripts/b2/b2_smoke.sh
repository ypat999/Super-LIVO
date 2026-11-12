#!/usr/bin/env bash
CFG=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/livox_360_visa_room_b2.yaml
bash /tmp/run_b2.sh b2sm40 "$CFG" 40 -p lio.visual.sigma_rot:=0.005 > /tmp/b2sm40.out 2>&1
echo "=== smoke 注入行样例:"
grep -E '\[VisRot\] inl=' /tmp/slio_b2sm40.log | tail -3
echo "=== none 行样例:"
grep -E '\[VisRot\] none' /tmp/slio_b2sm40.log | tail -2
echo "=== 接受率:"
grep -oE 'pairs=[0-9]+ ok=[0-9]+' /tmp/slio_b2sm40.log | tail -1
echo "=== 错误:"
grep -iE 'error|what\(\)|Segmentation|Exception|FATAL' /tmp/slio_b2sm40.log | head -5
