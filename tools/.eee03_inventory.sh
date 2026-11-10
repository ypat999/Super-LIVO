#!/bin/bash
for d in ntu_odom_eee_03_a ntu_odom_eee_03_b ntu_odom_eee_03_fl Retail_Street pcd; do
  echo "=== $d"
  ls /home/ywj/slam_data/$d 2>/dev/null | head -8
  if [ -f /home/ywj/slam_data/$d/metadata.yaml ]; then
    grep -E 'duration|message_count|name:' /home/ywj/slam_data/$d/metadata.yaml | head -12
  fi
done
echo "=== raw eee_03"
ls /home/ywj/slam_data/ntu_viral/raw/eee_03/ 2>/dev/null | head
echo "=== pcd/fl2_bag_lio"
ls /home/ywj/slam_data/pcd/fl2_bag_lio/ 2>/dev/null | head
echo "=== FAST-LIVO2 logs (official poses for eee_03)"
ls /home/ywj/git/Super-LIVO/third_party/FAST-LIVO2-ROS2/Log/result/ 2>/dev/null | grep -i eee_03
echo "=== 3d_map / grid_map"
ls /home/ywj/slam_data/3d_map /home/ywj/slam_data/grid_map 2>/dev/null | head -10
