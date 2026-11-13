#!/usr/bin/env bash
NEW=/home/ywj/slam_data/pl_official_ws/src/point_lio
echo "=== subscriptions in original laserMapping.cpp ==="
grep -nE "create_subscription|sub_" "$NEW/src/laserMapping.cpp" | head -20
echo "=== is livox_pcl_cbk referenced/active? ==="
grep -n "livox_pcl_cbk" "$NEW/src/laserMapping.cpp"
echo "=== README ==="
sed -n '1,80p' "$NEW/README.md"
echo "=== git log ==="
cd "$NEW" && git log --oneline -5
