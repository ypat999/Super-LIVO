#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
cd /home/ywj/git/Super-LIVO
python3 experiments/octvox_ablation/_probe_node_replay.py
