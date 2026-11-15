#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两份候选彩色图按"落盘顺序 6 等分"比色，用于取样策略对比。

用法: python3 _cmp_seg_two.py [a.pcd b.pcd ...]
默认对比 不封顶基线 vs patch_radius=1。
"""
import sys
import numpy as np
from scipy.spatial import cKDTree

REF = "/home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb_v2.pcd"
CASES = [
    ("基线(不封顶)", "/home/ywj/slam_data/replay_node_onrecord_nocap_20261115/super_livo_visual_a_colored.pcd"),
    ("patch_r1", "/home/ywj/slam_data/replay_patch1_20261115/super_livo_visual_a_colored.pcd"),
]
QN = 600000
SEGS = 6


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


def view(path):
    f, n, off = header(path)
    dt = np.dtype([(c, "<f4") for c in f])
    return np.memmap(path, dtype=dt, mode="r", offset=off, shape=(n,)), n


def rgb_of(arr):
    v = np.asarray(arr["rgb"]).view(np.uint32)
    return np.stack([(v >> 16) & 255, (v >> 8) & 255, v & 255], 1).astype(np.int16)


ref, nref = view(REF)
ref_xyz = np.stack([ref["x"], ref["y"], ref["z"]], 1).astype(np.float64)
ref_rgb = rgb_of(ref)
print("参考 %d 点，建 KDTree ..." % nref)
tree = cKDTree(ref_xyz)
del ref, ref_xyz

if len(sys.argv) > 1:
    CASES = [("argv%d" % i, p) for i, p in enumerate(sys.argv[1:])]

for name, path in CASES:
    case, nc = view(path)
    rng = np.random.default_rng(0)  # 与 _probe_range_effect.py 同种子，便于对比
    idx = np.sort(rng.choice(nc, size=min(QN, nc), replace=False))
    seg = (idx.astype(np.float64) / nc * SEGS).astype(np.int64).clip(0, SEGS - 1)
    pts = case[idx]
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
    rgb = rgb_of(pts)
    d, j = tree.query(xyz, k=1, workers=-1)
    ok = (d < 0.02) & (rgb.sum(1) > 0) & (ref_rgb[j].sum(1) > 0)
    err = np.full(len(xyz), np.nan)
    err[ok] = np.abs(rgb[ok] - ref_rgb[j][ok]).mean(1)
    print("\n=== %s  (%d 点) ===" % (name, nc))
    print("同位率 %.1f%%  MAE %.2f  MAE@d<2mm %.2f  p95 %.0f  >30%% %.2f%%" % (
        100.0 * ok.mean(), np.nanmean(err), np.nanmean(err[ok & (d < 0.002)]),
        np.nanpercentile(err, 95), 100.0 * np.nanmean(err > 30)))
    print("段    n      MAE   MAE<2mm  p95  >30%")
    for s in range(SEGS):
        m = seg == s
        e = err[m]
        e2 = err[m & (d < 0.002)]
        print("%-3d %6d  %5.2f  %7.2f  %5.0f  %4.1f%%" % (
            s, np.count_nonzero(m), np.nanmean(e),
            np.nanmean(e2) if np.isfinite(e2).any() else float("nan"),
            np.nanpercentile(e, 95), 100.0 * np.nanmean(e > 30)))
    del case, pts, xyz, rgb, d, j, ok, err
print("DONE_CMP_SEG_TWO")
