#!/bin/bash
# 串行链2：off 复跑（判定 off 是否逐位可复现）+ NTU eee_03 四组
R=/home/ywj/slam_data/room_slam
bash /tmp/p1_run.sh p1off3 off > /tmp/p1_off3.txt 2>&1
{
  echo "### off 复现性（最终二进制）"
  python3 /tmp/b3_bitcmp2.py $R/run_p1off3 $R/run_p1off2 2>&1 | tail -6
  python3 /tmp/p1_ndiff.py $R/run_p1off3 $R/run_p1off2 p1off3 p1off2 2>&1
  python3 /tmp/p1_ndiff.py $R/run_p1off2 $R/run_p1off p1off2 p1off(pre1651) 2>&1
  python3 /tmp/p1_ndiff.py $R/run_p1off $R/run_room_off p1off room_off 2>&1
  python3 /tmp/p1_ndiff.py $R/run_g_s1e4_t9 $R/run_p1off2 b2gated_s1e4_t9 off 2>&1
  python3 /tmp/p1_ndiff.py $R/run_c_b3only $R/run_p1off2 b3only off 2>&1
} > /tmp/p1_repro.txt 2>&1
bash /tmp/p1_ntu.sh offb -p lio.visual.enable:=false > /tmp/p1_ntu1.txt 2>&1
bash /tmp/p1_ntu.sh g_s1e4_t0 -p lio.visual.b2.enable:=true -p lio.visual.b3.enable:=false -p lio.visual.sigma_rot:=0.0001 -p lio.visual.chi2_thresh:=0.0 > /tmp/p1_ntu2.txt 2>&1
bash /tmp/p1_ntu.sh g_s1e4_t9 -p lio.visual.b2.enable:=true -p lio.visual.b3.enable:=false -p lio.visual.sigma_rot:=0.0001 -p lio.visual.chi2_thresh:=9.0 > /tmp/p1_ntu3.txt 2>&1
bash /tmp/p1_ntu.sh g_s3e4_t9 -p lio.visual.b2.enable:=true -p lio.visual.b3.enable:=false -p lio.visual.sigma_rot:=0.0003 -p lio.visual.chi2_thresh:=9.0 > /tmp/p1_ntu4.txt 2>&1
echo CHAIN2-DONE >> /tmp/p1_ntu4.txt
