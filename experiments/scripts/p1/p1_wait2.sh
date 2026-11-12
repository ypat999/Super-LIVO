#!/bin/bash
# 通用等待：bash /tmp/p1_wait2.sh <token> <file> <最大分钟>
TOK=$1
FILE=$2
MAX=${3:-9}
i=0
while [ "$i" -lt "$((MAX*6))" ]; do
  if grep -qa "$TOK" "$FILE" 2>/dev/null; then
    echo "DONE $TOK at $(date +%H:%M:%S)"
    exit 0
  fi
  i=$((i + 1))
  sleep 10
done
echo "STILL-RUNNING at $(date +%H:%M:%S)"
ls -dt /home/ywj/slam_data/ntu_viral/run_ntu_* /home/ywj/slam_data/room_slam/run_* 2>/dev/null | head -2
exit 1
