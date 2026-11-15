#!/usr/bin/env bash
L=/home/ywj/slam_data/replay_node_onrecord_20261115/vis.log
echo "frames=$(grep -ac '视觉帧' "$L")"
echo "overflow=$(grep -ac '队列溢出' "$L")"
echo "timeouts=$(grep -ac '等待图像超时' "$L")"
grep -a '视觉帧' "$L" | head -1
grep -a '视觉帧' "$L" | tail -1
grep -a '等待图像超时' "$L" | head -2
grep -a '等待图像超时' "$L" | tail -2
echo DONE_CHECK
