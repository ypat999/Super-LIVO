#!/usr/bin/env bash
echo "=== POINT-LIVO CMakeLists deps ==="
grep -nE "find_package|add_subdirectory|Sophus|Pangolin|pcl" /home/ywj/slam_data/livo_ws/src/POINT-LIVO/CMakeLists.txt | head -20
echo "=== POINT-LIVO submodules? ==="
cat /home/ywj/slam_data/livo_ws/src/POINT-LIVO/.gitmodules 2>/dev/null || echo none
ls /home/ywj/slam_data/livo_ws/src/POINT-LIVO/src
echo "=== Point-line-LIVO CMakeLists deps ==="
grep -nE "find_package|add_subdirectory|Sophus|Pangolin|pcl" /home/ywj/slam_data/livo_ws/src/Point-line-LIVO/CMakeLists.txt | head -20
echo "=== Point-line-LIVO submodules? ==="
cat /home/ywj/slam_data/livo_ws/src/Point-line-LIVO/.gitmodules 2>/dev/null || echo none
ls /home/ywj/slam_data/livo_ws/src/Point-line-LIVO/livo 2>/dev/null | head
echo "=== visual_a camera params used for this bag ==="
grep -nE "fx|fy|cx|cy|width|height|image" /home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml | head -15
echo "=== bag image resolution (from sqlite) ==="
python3 - <<'EOF'
import sqlite3
db=sqlite3.connect('/home/ywj/slam_data/Retail_Street/')
try:
    rows=db.execute("select name,type,count from topics t left join messages m on m.id=t.id group by name").fetchall()
except Exception:
    rows=db.execute("select name,type from topics").fetchall()
for r in rows: print(r)
EOF
