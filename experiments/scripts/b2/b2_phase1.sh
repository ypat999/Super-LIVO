#!/usr/bin/env bash
# B2 验证流水线 第一阶段：新 yaml 冒烟 + enable=false 与 off 配置逐帧等值性
CFGDIR=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config
B2=$CFGDIR/livox_360_visa_room_b2.yaml
OFF=/tmp/visa_ref/config/livox_360_visa_room_off.yaml
OUT=/home/ywj/slam_data/room_slam

echo "########## STEP1: smoke, new yaml (frame_slack=0.0875, max_features=300) ##########"
bash /tmp/run_b2.sh b2s2 "$B2" 40
echo "--- injected(applied) 行数: $(grep -cE '\[VisRot\] inl=' /tmp/slio_b2s2.log)"
grep -E '\[VisRot\] inl=' /tmp/slio_b2s2.log | tail -3
echo "--- 状态行:"
grep -E '\[VisRot\] none' /tmp/slio_b2s2.log | tail -3

echo
echo "########## STEP2a: b2 cfg + lio.visual.enable:=false ##########"
bash /tmp/run_b2.sh b2off "$B2" 40 -p lio.visual.enable:=false
echo "--- VisRot 行数(期望 0): $(grep -c '\[VisRot\]' /tmp/slio_b2off.log)"

echo
echo "########## STEP2b: off cfg ##########"
bash /tmp/run_b2.sh offsm "$OFF" 40
echo "--- VisRot 行数(期望 0): $(grep -c '\[VisRot\]' /tmp/slio_offsm.log)"

echo
echo "########## STEP2c: 逐帧等值性 ##########"
python3 /tmp/cmp_odom.py "$OUT/run_b2off" "$OUT/run_offsm"
echo "=== PHASE1 END ==="
