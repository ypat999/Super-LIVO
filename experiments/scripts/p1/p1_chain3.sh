#!/bin/bash
# 串行链3：NTU eee_03 四组（B2 门控验证不退化）
bash /tmp/p1_ntu.sh offb -p lio.visual.enable:=false > /tmp/p1_ntu1.txt 2>&1
echo "NTU-1-DONE" >> /tmp/p1_ntu1.txt
bash /tmp/p1_ntu.sh g_s1e4_t0 -p lio.visual.b2.enable:=true -p lio.visual.b3.enable:=false -p lio.visual.sigma_rot:=0.0001 -p lio.visual.chi2_thresh:=0.0 > /tmp/p1_ntu2.txt 2>&1
echo "NTU-2-DONE" >> /tmp/p1_ntu2.txt
bash /tmp/p1_ntu.sh g_s1e4_t9 -p lio.visual.b2.enable:=true -p lio.visual.b3.enable:=false -p lio.visual.sigma_rot:=0.0001 -p lio.visual.chi2_thresh:=9.0 > /tmp/p1_ntu3.txt 2>&1
echo "NTU-3-DONE" >> /tmp/p1_ntu3.txt
bash /tmp/p1_ntu.sh g_s3e4_t9 -p lio.visual.b2.enable:=true -p lio.visual.b3.enable:=false -p lio.visual.sigma_rot:=0.0003 -p lio.visual.chi2_thresh:=9.0 > /tmp/p1_ntu4.txt 2>&1
echo "CHAIN3-DONE" >> /tmp/p1_ntu4.txt
