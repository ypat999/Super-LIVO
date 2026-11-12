#!/bin/bash
# 串行链4：NTU eee_03 推荐档 σ=1e-3（有/无门控）
bash /tmp/p1_ntu.sh g_s1e3_t9 -p lio.visual.b2.enable:=true -p lio.visual.b3.enable:=false -p lio.visual.sigma_rot:=0.001 -p lio.visual.chi2_thresh:=9.0 > /tmp/p1_ntu5.txt 2>&1
echo "NTU-5-DONE" >> /tmp/p1_ntu5.txt
bash /tmp/p1_ntu.sh g_s1e3_t0 -p lio.visual.b2.enable:=true -p lio.visual.b3.enable:=false -p lio.visual.sigma_rot:=0.001 -p lio.visual.chi2_thresh:=0.0 > /tmp/p1_ntu6.txt 2>&1
echo "CHAIN4-DONE" >> /tmp/p1_ntu6.txt
