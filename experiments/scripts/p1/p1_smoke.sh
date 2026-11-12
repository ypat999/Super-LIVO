#!/bin/bash
# 40 s 冒烟：确认 B2 门控代码路径真的生效（reject/gate 行、参数行）
bash /tmp/b3_one.sh p1smoke 40 -p lio.visual.b2.enable:=true -p lio.visual.b3.enable:=false \
     -p lio.visual.sigma_rot:=0.0001 -p lio.visual.chi2_thresh:=9.0 > /tmp/p1_smoke_raw.txt 2>&1
L=/tmp/slio_p1smoke.log
echo "PARAM: $(grep -a '\[Param\] visual/b2' "$L" | head -1)"
echo "GATE lines: $(grep -ac 'VisB2gate' "$L")"
grep -a '\[VisB2gate\]' "$L" | tail -2
echo "REJECT lines: $(grep -ac 'VisB2. reject chi2' "$L")"
grep -a 'VisB2. reject chi2' "$L" | tail -3
echo "INL sample:"
grep -a '\[VisRot\] inl=' "$L" | tail -2
echo "ERRORS:"
grep -aiE 'error|assert|nan|Segmentation' "$L" | head -5
echo "SMOKE-DONE"
