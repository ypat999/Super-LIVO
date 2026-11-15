#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""几何受控的颜色比对：只在"节点点确实落在 R5 表面附近"的样本上比色。

之前的 _probe_node_vs_ref.py 按 2cm 体素比色，但节点是另一次 run，点位相对
R5 有 ~9mm 的系统偏移（见 _probe_nn_dist.py），大量点会落进相邻体素/边缘，
把颜色误差放大。这里改为：
  1) 对每个查询点找 R5 最近邻，距离 d < 阈值（默认 5mm）才认为"几何同位"；
  2) 只在这些点上比较颜色。
R0 与 R5 同 run（d=0），作为上界参照。
"""
import numpy as np
from scipy.spatial import cKDTree

REF = "/home/ywj/slam_data/official_vio_20261114/ladder/R5.pcd"
CASES = {
    "R0(离线A,同run)": "/home/ywj/slam_data/official_vio_20261114/ladder/R0.pcd",
    "fl2_A(节点)": "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_A/super_livo_visual_a_colored.pcd",
    "fl2_B(节点)": "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_B/super_livo_visual_b_colored.pcd",
}
QN = 400000
THR = 0.005


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
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float32)
    if "rgb" not in flds:
        return xyz, None
    v = arr["rgb"].view(np.uint32)
    rgb = np.stack([(v >> 16) & 255, (v >> 8) & 255, v & 255], 1).astype(np.uint8)
    return xyz, rgb


ref_xyz, ref_rgb = load(REF)
print("参考 R5 %d 点" % len(ref_xyz))
tree = cKDTree(ref_xyz.astype(np.float64))
del ref_xyz

for name, path in CASES.items():
    xyz, rgb = load(path, QN)
    d, idx = tree.query(xyz.astype(np.float64), k=1, workers=-1)
    ok = d < THR
    if not ok.any():
        print("%-18s 无同位点" % name)
        continue
    err = np.abs(rgb[ok].astype(np.int16) - ref_rgb[idx[ok]].astype(np.int16))
    e = err.mean(1)
    print("%-18s 同位(d<%.0fmm) %.1f%% | MAE %.2f  p95 %.0f  >30 %.2f%%"
          % (name, THR * 1000, 100.0 * ok.mean(), e.mean(),
             np.percentile(e, 95), 100.0 * (e > 30).mean()))
    for thr in (0.002, 0.005, 0.01, 0.02):
        m = d < thr
        if m.any():
            err2 = np.abs(rgb[m].astype(np.int16) - ref_rgb[idx[m]].astype(np.int16)).mean()
            print("      d<%.0fmm: 占比 %.1f%%  MAE %.2f"
                  % (thr * 1000, 100.0 * m.mean(), err2))
    del xyz, rgb, d, idx, ok
