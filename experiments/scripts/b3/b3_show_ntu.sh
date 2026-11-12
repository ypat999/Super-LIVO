#!/bin/bash
echo "===== /tmp/b3_run_ntu.sh ====="
cat /tmp/b3_run_ntu.sh 2>/dev/null
echo "===== existing ntu runs ====="
ls -d /home/ywj/slam_data/ntu_viral/run_ntu_* /tmp/slio_ntu_*.log 2>/dev/null
echo "===== b3_ntu_eval.py usage ====="
head -30 /tmp/b3_ntu_eval.py 2>/dev/null
