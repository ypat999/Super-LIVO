#!/usr/bin/env bash
echo "=== bag dir ==="
ls /home/ywj/slam_data/Retail_Street/
DB=$(ls /home/ywj/slam_data/Retail_Street/*.db3 | head -1)
echo "=== topics & counts ==="
python3 -c "
import sqlite3
db=sqlite3.connect('$DB')
for r in db.execute('select t.name,t.type,(select count(*) from messages m where m.topic_id=t.id) from topics t'):
    print(r)
"
echo "=== Point-line-LIVO livo/CMakeLists deps ==="
grep -nE "find_package|Sophus|Pangolin|onnx|opencv|pcl|Eigen" /home/ywj/slam_data/livo_ws/src/Point-line-LIVO/livo/CMakeLists.txt | head -15
echo "=== Point-line app deps ==="
grep -nE "find_package" /home/ywj/slam_data/livo_ws/src/Point-line-LIVO/app/CMakeLists.txt | head -10
echo "=== Point-line README run ==="
grep -nE "rosbag|roslaunch|source|catkin|clone|apt" /home/ywj/slam_data/livo_ws/src/Point-line-LIVO/README.md | head -20
echo "=== POINT-LIVO README run ==="
grep -nE "rosbag|roslaunch|source|catkin|clone|apt|Sophus|Pangolin" /home/ywj/slam_data/livo_ws/src/POINT-LIVO/README.md | head -20
