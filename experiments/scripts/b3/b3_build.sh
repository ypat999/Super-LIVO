#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /home/ywj/git/Super-LIVO/third_party/Super-LIO
colcon build --packages-select super_lio 2>&1 | tail -60
