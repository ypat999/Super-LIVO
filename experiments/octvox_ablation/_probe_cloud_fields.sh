#!/usr/bin/env bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
source /opt/ros/humble/setup.bash
python3 /home/ywj/git/Super-LIVO/experiments/octvox_ablation/_probe_cloud_fields.py
