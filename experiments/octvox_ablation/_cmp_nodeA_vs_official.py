#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""逐点比色：多个候选彩色图 vs 官方 LIVO 染色图。

用法: python3 _cmp_nodeA_vs_official.py [query.pcd ...]

口径：每个查询点取参考最近邻，先看几何一致性（NN 距离分位），
再只在几何同位的点上比色（避免相邻体素/边缘放大色差）。
"""
import sys
import numpy as np
from scipy.spatial import cKDTree

REF = "/home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb_v2.pcd"
DEFAULT = [
    ("同run_A节点", "/home/ywj/slam_data/replay_node_onrecord_20261115/super_livo_visual_a_colored.pcd"),
    ("官方染色参考", "/home/ywj/slam_data/colored_superlio_official_20261114/superlio_official_rgb.pcd"),
    ("fl2_A(早前run)", "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_A/super_livo_visual_a_colored.pcd"),
    ("fl2_B(早前run)", "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_B/super_livo_visual_b_colored.pcd"),
]
QN = 600000


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
    rgb = None
    key = "rgb" if "rgb" in flds else ("rgba" if "rgba" in flds else None)
    if key:
        v = np.asarray(arr[key]).view(np.uint32)
        rgb = np.stack([(v >> 16) & 255, (v >> 8) & 255, v & 255], 1).astype(np.int16)
    return xyz, rgb


ref_xyz, ref_rgb = load(REF)
print("参考(官方 v2): %d 点" % len(ref_xyz))
tree = cKDTree(ref_xyz)
del ref_xyz

cases = [(a, p) for a, p in DEFAULT]
if len(sys.argv) > 1:
    cases = [("argv%d" % i, p) for i, p in enumerate(sys.argv[1:])]

print("\n%-16s %8s %7s %7s %7s | %6s %6s %6s %6s | %6s | %7s %7s" %
      ("候选", "n", "p50mm", "p90mm", "d<10mm", "MAE", "p50", "p90", "p95", ">30%",
       "MAE<2mm", "MAE<5mm"))
for name, path in cases:
    try:
        xyz, rgb = load(path, QN)
    except Exception as exc:  # noqa: BLE001
        print("%-16s 读取失败: %s" % (name, exc))
        continue
    if rgb is None:
        print("%-16s 无 rgb 字段" % name)
        continue
    d, idx = tree.query(xyz, k=1, workers=-1)
    ok = (d < 0.02) & (rgb.sum(1) > 0) & (ref_rgb[idx].sum(1) > 0)
    e = np.abs(rgb[ok] - ref_rgb[idx[ok]]).mean(1)
    sub = {}
    for tag, thr in (("<2mm", 0.002), ("<5mm", 0.005)):
        m = ok & (d < thr)
        sub[tag] = np.abs(rgb[m] - ref_rgb[idx[m]]).mean() if m.any() else float("nan")
    print("%-16s %8d %7.2f %7.2f %6.1f%% | %6.2f %6.0f %6.0f %6.0f | %6.2f | %7.2f %7.2f" %
          (name, len(xyz), 1000 * np.percentile(d, 50), 1000 * np.percentile(d, 90),
           100.0 * (d < 0.01).mean(), e.mean(), np.percentile(e, 50),
           np.percentile(e, 90), np.percentile(e, 95), 100.0 * (e > 30).mean(),
           sub["<2mm"], sub["<5mm"]))
    del xyz, rgb, d, idx, ok, e
print("DONE_CMP_MULTI")
