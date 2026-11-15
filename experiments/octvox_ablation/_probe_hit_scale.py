#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""看节点产出与官方参考(R5)的几何吻合度随体素尺度的变化，判断 2cm 比色是否可信。"""
import numpy as np

REF = "/home/ywj/slam_data/official_vio_20261114/ladder/R5.pcd"
CASES = {
    "fl2_A": "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_A/super_livo_visual_a_colored.pcd",
    "fl2_B": "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_B/super_livo_visual_b_colored.pcd",
}


def load(path, maxn=None):
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
    off = len(txt)
    dt = np.dtype([(c, "<f4") for c in flds])
    arr = np.memmap(path, dtype=dt, mode="r", offset=off, shape=(n,))
    if maxn and n > maxn:
        idx = np.linspace(0, n - 1, maxn).astype(np.int64)
        arr = arr[idx]
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float32)
    return xyz


def keys(xyz, vs):
    k = np.floor(xyz.astype(np.float64) / vs).astype(np.int64)
    off = 1 << 20
    return np.unique(((k[:, 0] + off) << 42) | ((k[:, 1] + off) << 21) | (k[:, 2] + off))


rx = load(REF)
print("参考 %d 点" % len(rx))
refkeys = {}
for vs in (0.01, 0.02, 0.05, 0.10):
    refkeys[vs] = keys(rx, vs)
    print("  尺度 %.2fm -> %d 体素" % (vs, len(refkeys[vs])))
del rx

for name, path in CASES.items():
    xyz = load(path)
    print("== %s  %d 点" % (name, len(xyz)))
    for vs in (0.01, 0.02, 0.05, 0.10):
        k = keys(xyz, vs)
        hit = np.isin(k, refkeys[vs]).mean()
        print("   尺度 %.2fm 命中 %.1f%%" % (vs, 100.0 * hit))
    del xyz
