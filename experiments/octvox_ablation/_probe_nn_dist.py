#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""最近邻距离分布：判断"节点产出 vs 离线 R5"的差异是全局几何偏移，还是仅仅点采样不同。

若 median NN 距离 ~0（<1cm）且只有长尾，说明几何一致、坐标可比，2cm 比色可信；
若 median 明显 > 0（例如 2~5cm 系统性偏移），说明两次 run 的几何本身不重合，
任何基于坐标的比色都会被污染。
"""
import numpy as np
from scipy.spatial import cKDTree

REF = "/home/ywj/slam_data/official_vio_20261114/ladder/R5.pcd"
CASES = {
    "R0(离线A,同run)": "/home/ywj/slam_data/official_vio_20261114/ladder/R0.pcd",
    "fl2_A(节点,另一run)": "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_A/super_livo_visual_a_colored.pcd",
    "fl2_B(节点,另一run)": "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_B/super_livo_visual_b_colored.pcd",
}
QN = 300000


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
    dt = np.dtype([(c, "<f4") for c in flds])
    arr = np.memmap(path, dtype=dt, mode="r", offset=len(txt), shape=(n,))
    if maxn and n > maxn:
        arr = arr[np.linspace(0, n - 1, maxn).astype(np.int64)]
    return np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float32)


ref = load(REF)
print("参考 R5 全量 %d 点，建 KDTree ..." % len(ref))
tree = cKDTree(ref.astype(np.float64))
del ref

for name, path in CASES.items():
    q = load(path, QN)
    d, _ = tree.query(q.astype(np.float64), k=1, workers=-1)
    print("%-24s 查询 %d 点: median %.4f m, p90 %.4f, p99 %.4f, >5cm %.1f%%"
          % (name, len(q), np.median(d), np.percentile(d, 90),
             np.percentile(d, 99), 100.0 * (d > 0.05).mean()))
