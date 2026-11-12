#!/bin/bash
# A 组：enable=false 逐位回归 + σ=1e-4 无门控危险基线
bash /tmp/p1_run.sh p1off off
bash /tmp/p1_run.sh p1_s1e4_t0 b2 -p lio.visual.sigma_rot:=0.0001 -p lio.visual.chi2_thresh:=0.0
echo "===== BITCMP p1off vs run_room_off (最终二进制)"
python3 /tmp/b3_bitcmp2.py /home/ywj/slam_data/room_slam/run_p1off /home/ywj/slam_data/room_slam/run_room_off
echo "GROUPA-DONE"
