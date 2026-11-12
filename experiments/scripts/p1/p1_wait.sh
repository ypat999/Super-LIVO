#!/bin/bash
# 轮询等待 room 串行链完成：bash /tmp/p1_wait.sh <最大分钟>
MAX=${1:-10}
i=0
while [ "$i" -lt "$((MAX*6))" ]; do
  if grep -qa CHAIN-DONE /tmp/p1_off2.txt 2>/dev/null; then
    echo "CHAIN-FINISHED at $(date +%H:%M:%S)"
    exit 0
  fi
  i=$((i + 1))
  sleep 10
done
echo "CHAIN-STILL-RUNNING at $(date +%H:%M:%S) last-run: $(ls -dt /home/ywj/slam_data/room_slam/run_* | head -1)"
exit 1
