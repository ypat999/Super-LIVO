#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按落盘顺序分段比色：检验 VoxelColorMedian 的 max_entries(3e6) 是否早早饱和。

节点把彩色点按时间顺序 append 到 colored_map_，所以 PCD 前段 = 轨迹早期、
后段 = 后期。若中位数表在 300 万条目处饱和，则只有早期点享受到"时间中位色"，
后期点只能保留单帧色 → 色差应随段落单调上升。
"""
import numpy as np
from scipy.spatial import cKDTree

REF = "/home/ywj/slam_data/official_vio_20261114/ladder/R5.pcd"
CASE = "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_A/super_livo_visual_a_colored.pcd"
CHUNK = 2000000
THR = 0.005


def header(path):
    with open(path, "rb") as f:
        txt = b""
        while not txt.endswith(b"DATA binary\n"):
            c = f.read(1)
            if not c:
                break
            txt += c
    lines = txt.decode("ascii", "ignore").splitlines()
    flds = [l.split()[1:] for l in lines if l.startswith("FIELDS")][0]
    n = int([l.split()[1] for l in lines if l.startswith("POINTS")][0])
    return flds, n, len(txt)


def view(path, flds, n, off):
    dt = np.dtype([(c, "<f4") for c in flds])
    return np.memmap(path, dtype=dt, mode="r", offset=off, shape=(n,))


def rgb_of(arr):
    v = arr["rgb"].view(np.uint32)
    return np.stack([(v >> 16) & 255, (v >> 8) & 255, v & 255], 1).astype(np.uint8)


f, n, off = header(REF)
ref = view(REF, f, n, off)
ref_xyz = np.stack([ref["x"], ref["y"], ref["z"]], 1).astype(np.float32)
ref_rgb = rgb_of(ref)
print("参考 R5 %d 点，建 KDTree..." % n)
tree = cKDTree(ref_xyz.astype(np.float64))
del ref, ref_xyz

f, n, off = header(CASE)
arr = view(CASE, f, n, off)
print("节点 A %d 点，按顺序分 %d 段" % (n, (n + CHUNK - 1) // CHUNK))
for s in range(0, n, CHUNK):
    e = min(n, s + CHUNK)
    seg = arr[s:e]
    xyz = np.stack([seg["x"], seg["y"], seg["z"]], 1).astype(np.float32)
    rgb = rgb_of(seg)
    d, idx = tree.query(xyz.astype(np.float64), k=1, workers=-1)
    ok = d < THR
    if ok.any():
        err = np.abs(rgb[ok].astype(np.int16) - ref_rgb[idx[ok]].astype(np.int16)).mean()
        print("  [%8d,%8d) 同位 %.1f%%  MAE %.2f" % (s, e, 100.0 * ok.mean(), err))
    del seg, xyz, rgb, d, idx, ok
