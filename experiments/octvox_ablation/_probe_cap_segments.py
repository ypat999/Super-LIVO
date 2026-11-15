#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按落盘顺序（= 时间顺序）分段比色：对比"3e6 封顶"与"不封顶"两次录制流回放。

节点把彩色点按时间顺序 append 到 colored_map_，PCD 前段 = 轨迹早期。
中位数表若在 3e6 条目饱和，只有早期点能吃到"时间中位色"，色差应随段落上升。
用法: python3 _probe_cap_segments.py
"""
import numpy as np
from scipy.spatial import cKDTree

REF = "/home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb_v2.pcd"
CASES = [
    ("封顶3e6", "/home/ywj/slam_data/replay_node_onrecord_20261115/super_livo_visual_a_colored.pcd"),
    ("不封顶", "/home/ywj/slam_data/replay_node_onrecord_nocap_20261115/super_livo_visual_a_colored.pcd"),
]
SEGS = 6
QN_PER_SEG = 100000


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
    v = np.asarray(arr["rgb"]).view(np.uint32)
    return np.stack([(v >> 16) & 255, (v >> 8) & 255, v & 255], 1).astype(np.int16)


f, n, off = header(REF)
ref = view(REF, f, n, off)
ref_xyz = np.stack([ref["x"], ref["y"], ref["z"]], 1).astype(np.float64)
ref_rgb = rgb_of(ref)
print("参考(官方 v2) %d 点，建 KDTree..." % n)
tree = cKDTree(ref_xyz)
del ref, ref_xyz

rng = np.random.default_rng(0)
tables = {}
for name, path in CASES:
    f, n, off = header(path)
    arr = view(path, f, n, off)
    print("\n%s: %d 点" % (name, n))
    bounds = np.linspace(0, n, SEGS + 1).astype(np.int64)
    rows = []
    for s in range(SEGS):
        idx = np.sort(rng.choice(np.arange(bounds[s], bounds[s + 1]),
                                 size=min(QN_PER_SEG, bounds[s + 1] - bounds[s]),
                                 replace=False))
        seg = arr[idx]
        xyz = np.stack([seg["x"], seg["y"], seg["z"]], 1).astype(np.float64)
        rgb = rgb_of(seg)
        d, j = tree.query(xyz, k=1, workers=-1)
        ok = (d < 0.02) & (rgb.sum(1) > 0) & (ref_rgb[j].sum(1) > 0)
        e = np.abs(rgb[ok] - ref_rgb[j][ok]).mean(1)
        m2 = ok & (d < 0.002)
        e2 = np.abs(rgb[m2] - ref_rgb[j][m2]).mean() if m2.any() else float("nan")
        rows.append((s, 100.0 * ok.mean(), e.mean(), np.percentile(e, 95),
                     100.0 * np.mean(e > 30), e2))
        print("  段%d [%8d,%8d) 同位 %4.1f%%  MAE %5.2f  p95 %4.0f  >30 %4.1f%%  MAE<2mm %5.2f" %
              (s, bounds[s], bounds[s + 1], rows[-1][1], rows[-1][2], rows[-1][3],
               rows[-1][4], rows[-1][5]))
        del seg, xyz, rgb, d, j, ok, e
    tables[name] = rows
    del arr

print("\n段落趋势（MAE）:")
print("%-10s" % "case" + "".join("  段%d" % s for s in range(SEGS)))
for name, rows in tables.items():
    print("%-10s" % name + "".join(" %5.2f" % r[2] for r in rows))
print("\n段落趋势（MAE@<2mm）:")
for name, rows in tables.items():
    print("%-10s" % name + "".join(" %5.2f" % r[5] for r in rows))
print("DONE_CAP_SEGMENTS")
