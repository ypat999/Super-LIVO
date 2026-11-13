#!/usr/bin/env bash
cd /home/ywj/slam_data/pl_official_ws/src/point_lio
git fetch --unshallow 2>&1 | tail -1
echo "=== history of livox_pcl_cbk ==="
git log --oneline -S "livox_pcl_cbk" -- src/laserMapping.cpp
echo "=== when commented out ==="
git log --oneline -15
echo "=== check each: count active (uncommented) lines of livox_pcl_cbk ==="
for c in $(git log --format=%h -S "livox_pcl_cbk" -- src/laserMapping.cpp); do
  n=$(git show "$c:src/laserMapping.cpp" 2>/dev/null | grep -vc "^// void livox_pcl_cbk" )
  active=$(git show "$c:src/laserMapping.cpp" 2>/dev/null | grep -n "void livox_pcl_cbk" | grep -v "^\s*[0-9]*://" | head -1)
  echo "$c -> $active"
done
