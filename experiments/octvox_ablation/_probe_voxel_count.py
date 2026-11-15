#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统计若干 PCD 在 1cm 体素下的唯一体素数，用于判断 VoxelColorMedian 的
max_entries 上限（原 3e6）是否饱和。
用法: python3 _probe_voxel_count.py [pcd ...]
"""
import sys
import numpy as np


def header(path):
    with open(path, "rb") as f:
        txt = b""
        while not txt.endswith(b"DATA binary\n"):
            c = f.read(1)
            if not c:
                break
            txt += c
    return txt


def load_xyz(path):
    txt = header(path)
    lines = txt.decode("ascii", "ignore").splitlines()
    flds = [l.split()[1:] for l in lines if l.startswith("FIELDS")][0]
    n = int([l.split()[1] for l in lines if l.startswith("POINTS")][0])
    dt = np.dtype([(c, "<f4") for c in flds])
    arr = np.memmap(path, dtype=dt, mode="r", offset=len(txt), shape=(n,))
    return np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float64)


DEFAULT = [
    "/home/ywj/slam_data/replay_node_onrecord_20261115/super_livo_visual_a_colored.pcd",
    "/home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb_v2.pcd",
]

for path in (sys.argv[1:] or DEFAULT):
    xyz = load_xyz(path)
    key = np.floor(xyz * 100.0).astype(np.int32)
    uniq = np.unique(key, axis=0)
    print("%s\n  点=%d 1cm体素=%d  体素/点=%.2f  估算条目内存≈%.0f MB" % (
        path, len(xyz), len(uniq), len(uniq) / len(xyz), len(uniq) * 60 / 1e6))
    del xyz, key, uniq
print("DONE_VOXEL_COUNT")
