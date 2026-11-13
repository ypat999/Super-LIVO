#!/usr/bin/env bash
echo "########## POINT-LIVO ##########"
P=/home/ywj/slam_data/livo_ws/src/POINT-LIVO
echo "--- configs list ---"
ls "$P/config"
echo "--- retail avia config ---"
cat "$P/config/avia_retail.yaml" 2>/dev/null || echo "(no avia_retail.yaml)"
echo "--- camera_pinhole_retail.yaml ---"
cat "$P/config/camera_pinhole_retail.yaml" 2>/dev/null
echo "--- launch files ---"
ls "$P/launch"
echo "--- subscribed topics ---"
grep -nE "subscribe" "$P/src/LIVMapper.cpp" | head
echo "--- pcd save ---"
grep -nE "PCD|pcd|save" "$P/src/LIVMapper.cpp" | head -15
echo "########## Point-line-LIVO ##########"
Q=/home/ywj/slam_data/livo_ws/src/Point-line-LIVO
echo "--- r3live.yaml ---"
sed -n '1,60p' "$Q/config/r3live.yaml"
echo "--- launch ---"
ls "$Q/launch"
echo "--- subscribed topics ---"
grep -rnE "subscribe" "$Q/livo" --include=*.cpp | head
echo "--- deps in CMakeLists ---"
grep -nE "find_package" "$Q/CMakeLists.txt" | head -15
