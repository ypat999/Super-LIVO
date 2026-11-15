#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判断"两次独立 run 的世界系"是否可直接按坐标比较。

思路：fl2_A 与 fl2_B 是同一段 bag 的两次独立运行。若它们之间按世界坐标的体素
命中率也很低，说明两次 run 的世界系原点/朝向不同 -> 任何跨 run 的坐标比对
（包括与离线 ladder R5 的 2cm 比色）都无效，必须先在同一个世界系里比。
对照：R0 vs R5 属于同一次离线运行，命中率应接近 100%。
"""
import numpy as np

PAIRS = [
    ("R5", "/home/ywj/slam_data/official_vio_20261114/ladder/R5.pcd",
     "R0", "/home/ywj/slam_data/official_vio_20261114/ladder/R0.pcd"),
    ("fl2_A", "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_A/super_livo_visual_a_colored.pcd",
     "fl2_B", "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_B/super_livo_visual_b_colored.pcd"),
    ("R5", "/home/ywj/slam_data/official_vio_20261114/ladder/R5.pcd",
     "fl2_B", "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_B/super_livo_visual_b_colored.pcd"),
]


def load(path, maxn=2000000):
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
    dt = np.dtype([(c, "<f4") for c in flds])
    arr = np.memmap(path, dtype=dt, mode="r", offset=len(txt), shape=(n,))
    if n > maxn:
        arr = arr[np.linspace(0, n - 1, maxn).astype(np.int64)]
    return np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float32)


def keys(xyz, vs):
    k = np.floor(xyz.astype(np.float64) / vs).astype(np.int64)
    off = 1 << 20
    return np.unique(((k[:, 0] + off) << 42) | ((k[:, 1] + off) << 21) | (k[:, 2] + off))


for na, pa, nb, pb in PAIRS:
    a, b = load(pa), load(pb)
    print("== %s(%d) vs %s(%d)" % (na, len(a), nb, len(b)))
    print("   centroid %s=%s  %s=%s" % (na, np.round(a.mean(0), 3), nb, np.round(b.mean(0), 3)))
    print("   min %s=%s  %s=%s" % (na, np.round(a.min(0), 2), nb, np.round(b.min(0), 2)))
    print("   max %s=%s  %s=%s" % (na, np.round(a.max(0), 2), nb, np.round(b.max(0), 2)))
    for vs in (0.02, 0.10, 0.30):
        ka, kb = keys(a, vs), keys(b, vs)
        hit = np.isin(kb, ka).mean()
        print("   尺度 %.2fm  %s 体素落在 %s 内的比例 %.1f%%" % (vs, nb, na, 100.0 * hit))
    del a, b
