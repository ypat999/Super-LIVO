#!/bin/bash
# 最终二进制下的 enable=false 逐位相同验收
set -e
bash /tmp/b3_one.sh b3off2 0 -p lio.visual.enable:=false 2>&1 | grep -aE 'PRE-CLEAN|POST-CLEAN|### EVAL|回环|终点|帧数|Param] visual|ONE-DONE' | head -20
echo "===== BITCMP b3off2 vs run_room_off"
python3 /tmp/b3_bitcmp2.py /home/ywj/slam_data/room_slam/run_b3off2 /home/ywj/slam_data/room_slam/run_room_off
echo "===== BITCMP b3off2 vs run_b3off (旧二进制同配置)"
python3 /tmp/b3_bitcmp2.py /home/ywj/slam_data/room_slam/run_b3off2 /home/ywj/slam_data/room_slam/run_b3off
grep -a 'Param] visual' /tmp/slio_b3off2.log
echo "BITCHECK-DONE"
