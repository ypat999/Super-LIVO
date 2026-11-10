#!/bin/bash
echo "=== retail_street_livo_poses.txt 首末 + 行数"
for f in retail_street_livo_poses retail_street_livo_poses_rel retail_street_livo_poses_v2; do
  p=/home/ywj/slam_data/pcd/retail_street_livo_official/$f.txt
  echo "-- $f: $(wc -l < $p) 行"
  head -1 "$p"; tail -1 "$p"
done
echo "=== superlio poses txt"
head -1 /home/ywj/slam_data/pcd/retail_street_superlio_poses.txt
tail -1 /home/ywj/slam_data/pcd/retail_street_superlio_poses.txt
echo "  行数 $(wc -l < /home/ywj/slam_data/pcd/retail_street_superlio_poses.txt)"
echo "=== replay_odom.txt (可能是 sl_frames odom 导出)"
head -2 /home/ywj/slam_data/pcd/replay_odom.txt; tail -1 /home/ywj/slam_data/pcd/replay_odom.txt
echo "=== fl2 lidar_poses.txt"
head -1 /home/ywj/slam_data/pcd/fl2_bag_lio/lidar_poses.txt; tail -1 /home/ywj/slam_data/pcd/fl2_bag_lio/lidar_poses.txt
echo "  行数 $(wc -l < /home/ywj/slam_data/pcd/fl2_bag_lio/lidar_poses.txt)"
echo "=== 各 rgb/pcd 成品头部字段与点数（判断官方在 retail 是否真有上色）"
for p in /home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb.pcd /home/ywj/slam_data/pcd/retail_street_superlio_poses.txt; do
  echo "-- $p"; head -c 400 "$p" 2>/dev/null | tr -d '\0' | grep -iE 'FIELDS|POINTS|WIDTH' | head
done
