#!/bin/bash
src=/home/ywj/slam_data/singleframe/blackframe_replay/attribution/high_jump_voxels
dst=/mnt/d/traetmp/high_jump_voxels
mkdir -p "$dst"
# 按文件名排序复制前 6 张（文件名即 voxel_id，脚本里已按严重度生成，全部 50 张一并同步）
cp -f "$src"/*.png "$dst"/ 2>/dev/null
ls "$dst" | head -8
ls "$dst" | wc -l
