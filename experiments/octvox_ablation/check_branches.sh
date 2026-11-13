#!/usr/bin/env bash
echo "=== unitreerobotics/point_lio_unilidar branches ==="
curl -s 'https://api.github.com/repos/unitreerobotics/point_lio_unilidar/branches?per_page=50' | grep '"name"'
echo "=== hku-mars/Point-LIO branches ==="
curl -s 'https://api.github.com/repos/hku-mars/Point-LIO/branches?per_page=50' | grep '"name"'
echo "=== dfloreaa/point_lio_ros2 branches ==="
curl -s 'https://api.github.com/repos/dfloreaa/point_lio_ros2/branches?per_page=50' | grep '"name"'
