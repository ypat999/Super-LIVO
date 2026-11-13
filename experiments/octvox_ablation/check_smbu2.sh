#!/usr/bin/env bash
echo "=== SMBU grid_map_ros2 branch: package.xml ==="
curl -s https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/point_lio/grid_map_ros2/package.xml | grep -E "<name>|format|depend" | head -20
echo "=== SMBU grid_map_ros2: livox cbk active? ==="
curl -s https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/point_lio/grid_map_ros2/src/laserMapping.cpp | grep -nE "livox_pcl_cbk|create_subscription|CustomMsg" | head -10
echo "=== SMBU grid_map_ros2: preprocess avia handler? ==="
curl -s https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/point_lio/grid_map_ros2/src/preprocess.cpp | grep -nE "avia_handler|case AVIA|CustomMsg" | head -10
echo "=== more GitHub search: point-lio ros2 ==="
curl -s 'https://api.github.com/search/repositories?q=Point-LIO+ros2&sort=stars&order=desc&per_page=10' | grep -E '"full_name"|"stargazers_count"' | head -20
