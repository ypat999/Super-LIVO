#!/usr/bin/env python3
"""核对打了时基补丁后，/aft_mapped_to_init 的 header.stamp 是否等于 lidar 帧时刻。

用法：source /opt/ros/humble/setup.bash && \
      python3 04_check_odom_timebase.py <odom bag 目录> <同轮的 pose.txt>
"""
import importlib.util
import sys

import numpy as np

spec = importlib.util.spec_from_file_location(
    "B", "/home/ywj/git/Super-LIVO/experiments/octvox_ablation/01_build_maps.py")
B = importlib.util.module_from_spec(spec)
spec.loader.exec_module(B)

bagdir, pose = sys.argv[1], sys.argv[2]
od = B.read_bag(bagdir, ["/aft_mapped_to_init"])[
    "/aft_mapped_to_init"]
to = np.array([t for t, _ in od])
tp = np.loadtxt(pose)[:, 0]
print(f"odom {len(to)} 条  header {to[0]:.3f} → {to[-1]:.3f}  span {to[-1]-to[0]:.1f} s")
print(f"pose {len(tp)} 条  lidar  {tp[0]:.3f} → {tp[-1]:.3f}  span {tp[-1]-tp[0]:.1f} s")
nn = np.abs(np.subtract.outer(to, tp)).min(axis=1)
print(f"每条 odom header 到最近 pose 时刻的差：中位 {np.median(nn)*1e3:.2f} ms "
      f"P90 {np.percentile(nn,90)*1e3:.2f} max {nn.max()*1e3:.2f} ms")
print("判定：" + ("header 已落在 lidar 时基（补丁生效）" if np.median(nn) < 2e-3
                 else "header 仍是墙钟（补丁未生效）"))
