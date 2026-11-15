#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""交叉比较两次 FL2 run 的 A / B 彩色地图几何。

若两者只是"同一套输入 + 不同着色"，几何应逐点重合；
若 NN 距离随位置增长，说明两次 FL2 建图本身不同（输入不同）。
用法: python3 _cmp_ab_pcd_geom.py
"""
import sys
import numpy as np
from scipy.spatial import cKDTree

A = "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_A/super_livo_visual_a_colored.pcd"
B = "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_B/super_livo_visual_b_colored.pcd"
if len(sys.argv) >= 3:
    A, B = sys.argv[1], sys.argv[2]
QN = 300000


def header(path):
    with open(path, "rb") as f:
        txt = b""
        while not txt.endswith(b"DATA binary\n"):
            c = f.read(1)
            if not c:
                break
            txt += c
    return txt


def load(path, maxn=None):
    txt = header(path)
    lines = txt.decode("ascii", "ignore").splitlines()
    flds = [l.split()[1:] for l in lines if l.startswith("FIELDS")][0]
    n = int([l.split()[1] for l in lines if l.startswith("POINTS")][0])
    dt = np.dtype([(c, "<f4") for c in flds])
    arr = np.memmap(path, dtype=dt, mode="r", offset=len(txt), shape=(n,))
    if maxn and n > maxn:
        arr = arr[np.linspace(0, n - 1, maxn).astype(np.int64)]
    return np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float64), flds


pa, fa = load(A)
pb, fb = load(B)
print("A: %d 点 fields=%s" % (len(pa), fa))
print("B: %d 点 fields=%s" % (len(pb), fb))
print("A bbox min=%s max=%s" % (pa.min(0).round(3), pa.max(0).round(3)))
print("B bbox min=%s max=%s" % (pb.min(0).round(3), pb.max(0).round(3)))
print("质心 A-B (mm) = %s" % ((pa.mean(0) - pb.mean(0)) * 1000).round(2))

rng = np.random.default_rng(0)
idx = rng.choice(len(pa), size=min(QN, len(pa)), replace=False)
q = pa[idx]
del pa

tree = cKDTree(pb)
d, j = tree.query(q, k=1, workers=-1)
mm = d * 1000.0
print("\nA->B 最近邻 (mm): p50=%.2f p90=%.2f p95=%.2f p99=%.2f" % (
    np.percentile(mm, 50), np.percentile(mm, 90), np.percentile(mm, 95),
    np.percentile(mm, 99)))
for t in (2, 5, 10, 20, 50, 100):
    print("  d<%3dmm: %5.1f%%" % (t, 100.0 * np.mean(mm < t)))

off = (q - pb[j]) * 1000.0
print("\n有符号偏移 A-B (mm): 中位数 dx=%.2f dy=%.2f dz=%.2f | 均值 dx=%.2f dy=%.2f dz=%.2f" % (
    tuple(np.median(off, 0)) + tuple(np.mean(off, 0))))

bins = np.linspace(q[:, 0].min(), q[:, 0].max(), 7)
print("\n按 x 分箱:")
for lo, hi in zip(bins[:-1], bins[1:]):
    m = (q[:, 0] >= lo) & (q[:, 0] < hi)
    if np.count_nonzero(m) < 500:
        continue
    print("  x[%7.2f,%7.2f) n=%6d  p50=%.2f p90=%.2f  偏移中位 dx=%+.2f dy=%+.2f dz=%+.2f" % (
        lo, hi, np.count_nonzero(m), np.percentile(mm[m], 50),
        np.percentile(mm[m], 90), *np.median(off[m], 0)))

bins = np.linspace(q[:, 2].min(), q[:, 2].max(), 6)
print("\n按 z 分箱:")
for lo, hi in zip(bins[:-1], bins[1:]):
    m = (q[:, 2] >= lo) & (q[:, 2] < hi)
    if np.count_nonzero(m) < 500:
        continue
    print("  z[%7.2f,%7.2f) n=%6d  p50=%.2f p90=%.2f" % (
        lo, hi, np.count_nonzero(m), np.percentile(mm[m], 50),
        np.percentile(mm[m], 90)))
print("DONE_CMP_AB_GEOM")
