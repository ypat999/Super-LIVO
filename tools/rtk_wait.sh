#!/usr/bin/env bash
# 等待一次 run 的哨兵文件（run_rtk.sh 结束时 touch /tmp/rtk_<tag>.done）
TAG=${1:?需要 tag}
for i in $(seq 1 45); do
  [ -f /tmp/rtk_$TAG.done ] && { echo "ready after ${i} polls"; break; }
  sleep 15
done
echo "sentinel: $(ls -l /tmp/rtk_$TAG.done 2>/dev/null || echo MISSING)"
grep -a -E 'odom reset|Watchdog' /tmp/slio_$TAG.log | tail -3
