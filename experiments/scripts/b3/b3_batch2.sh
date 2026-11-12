#!/bin/bash
# B3 阶段2：room 网格 B2/B3。全部完整 160s。阻塞等待跑批锁。
set +u
exec 9>/tmp/ros_batch.lock
flock 9
Y=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config/livox_360_visa_room_b3.yaml
B=/tmp/run_b2.sh

bash $B b2full   $Y 0 -p lio.visual.b3.enable:=false
bash $B bd05s005 $Y 0 -p lio.visual.b3.baseline_dt:=0.5
bash $B bd10s005 $Y 0 -p lio.visual.b3.baseline_dt:=1.0
bash $B bd20s005 $Y 0 -p lio.visual.b3.baseline_dt:=2.0
bash $B bd10s002 $Y 0 -p lio.visual.b3.baseline_dt:=1.0 -p lio.visual.b3.sigma_rot:=0.02
bash $B bd10s010 $Y 0 -p lio.visual.b3.baseline_dt:=1.0 -p lio.visual.b3.sigma_rot:=0.10

echo "================= EVAL ================="
for t in b2full bd05s005 bd10s005 bd20s005 bd10s002 bd10s010; do
  echo "-- $t"
  python3 /tmp/eval_loop.py /home/ywj/slam_data/room_slam/run_$t $t 2>&1 | head -5
  grep -a '\[VisB3\]' /tmp/slio_$t.log | tail -1
done
echo BATCH2-DONE
