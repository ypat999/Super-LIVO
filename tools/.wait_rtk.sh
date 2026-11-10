#!/usr/bin/env bash
# 等待 construction_seq2.db3 下载完成（以 .sha256 落盘为标志），然后校验并 ros2 bag info。
D=/home/ywj/slam_data/rtk_slam/construction_seq2
EXPECT=10656112640
for i in $(seq 1 80); do
  [ -s "$D/construction_seq2.db3.sha256" ] && break
  sleep 15
done
sz=$(stat -c %s "$D/construction_seq2.db3" 2>/dev/null || echo 0)
echo "size=$sz expect=$EXPECT"
[ "$sz" = "$EXPECT" ] && echo "SIZE OK" || echo "SIZE MISMATCH（需重跑 tools/.dl_rtk_slam.sh 续传）"
echo "=== sha256 ==="; cat "$D/construction_seq2.db3.sha256" 2>/dev/null
echo "=== ros2 bag info ==="
source /opt/ros/humble/setup.bash
ros2 bag info "$D" 2>&1 | tail -22
