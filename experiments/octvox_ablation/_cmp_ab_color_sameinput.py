#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""同输入（同一段录制流）下 A 节点 vs B 节点的着色差异。

几何已验证逐点重合，这里只在 NN<1mm 的共用点上比 RGB，
直接回答"A/B 的着色链路有没有差别"。
用法: python3 _cmp_ab_color_sameinput.py [A.pcd B.pcd]
"""
import sys
import numpy as np
from scipy.spatial import cKDTree

A = "/home/ywj/slam_data/replay_node_onrecord_20261115/super_livo_visual_a_colored.pcd"
B = "/home/ywj/slam_data/replay_node_onrecord_B_20261115/super_livo_visual_b_colored.pcd"
if len(sys.argv) >= 3:
    A, B = sys.argv[1], sys.argv[2]
QN = 400000


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
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float64)
    v = np.asarray(arr["rgb"]).view(np.uint32)
    rgb = np.stack([(v >> 16) & 255, (v >> 8) & 255, v & 255], 1).astype(np.int16)
    return xyz, rgb


xa, ca = load(A, QN)
xb, cb = load(B)
print("A 抽样 %d / B 全量 %d" % (len(xa), len(xb)))

tree = cKDTree(xb)
d, j = tree.query(xa, k=1, workers=-1)
mm = d * 1000.0
m = (mm < 1.0) & (ca.sum(1) > 0) & (cb[j].sum(1) > 0)
print("共用点 (NN<1mm 且两侧都有色): %d (%.1f%%)" % (
    np.count_nonzero(m), 100.0 * np.count_nonzero(m) / len(xa)))

ea = np.abs(ca[m].astype(np.float64) - cb[j][m].astype(np.float64))
per = ea.mean(1)
print("\nA vs B 颜色 (共用点):")
print("  MAE=%.2f  p50=%.0f  p90=%.0f  p95=%.0f  >30 占比=%.2f%%" % (
    per.mean(), np.percentile(per, 50), np.percentile(per, 90),
    np.percentile(per, 95), 100.0 * np.mean(per > 30)))
print("  有符号偏差 A-B: R=%+.2f G=%+.2f B=%+.2f" % tuple(
    (ca[m].astype(np.float64) - cb[j][m].astype(np.float64)).mean(0)))
for k, name in enumerate("RGB"):
    dif = ca[m][:, k].astype(np.float64) - cb[j][m][:, k].astype(np.float64)
    print("  %s: MAE=%.2f  |diff|>10 占比=%.1f%%" % (
        name, np.abs(dif).mean(), 100.0 * np.mean(np.abs(dif) > 10)))

print("\n严格相同颜色的比例: %.1f%%" % (100.0 * np.mean(per == 0)))
print("差值分布 (每点三通道平均):")
for t in (1, 2, 5, 10, 20, 50):
    print("  <=%3d: %5.1f%%  >%3d: %5.1f%%" % (
        t, 100.0 * np.mean(per <= t), t, 100.0 * np.mean(per > t)))
print("DONE_CMP_AB_COLOR_SAMEINPUT")
