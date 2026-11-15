#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把节点产出的 A/B 彩色 PCD 与官方染色参考(R5)按 2cm 体素比对颜色误差。"""
import sys
import numpy as np

REF = "/home/ywj/slam_data/official_vio_20261114/ladder/R5.pcd"
CASES = {
    "R0(离线A全量)": "/home/ywj/slam_data/official_vio_20261114/ladder/R0.pcd",
    "R1(离线A无中位)": "/home/ywj/slam_data/official_vio_20261114/ladder/R1.pcd",
    "fl2_A(节点官方位姿)": "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_A/super_livo_visual_a_colored.pcd",
    "fl2_B(节点官方位姿)": "/home/ywj/slam_data/colored_fl2_vio_20261115/fl2_B/super_livo_visual_b_colored.pcd",
}
VS = 0.02


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
    rgb = arr["rgb"].view(np.uint32)
    r = (rgb >> 16) & 255
    g = (rgb >> 8) & 255
    b = rgb & 255
    return xyz, np.stack([r, g, b], 1).astype(np.uint8)


def key(xyz):
    k = np.floor(xyz.astype(np.float64) / VS).astype(np.int64)
    off = 1 << 20
    return ((k[:, 0] + off) << 42) | ((k[:, 1] + off) << 21) | (k[:, 2] + off)


print("读参考:", REF)
rx, rc = load(REF)
rk = key(rx)
rk, inv = np.unique(rk, return_inverse=True)
acc = np.zeros((len(rk), 3), np.float64)
for c in range(3):
    acc[:, c] = np.bincount(inv, weights=rc[:, c], minlength=len(rk))
cnt = np.bincount(inv, minlength=len(rk))
acc /= np.maximum(cnt, 1)[:, None]
print("  参考 %d 点 -> %d 个 2cm 体素" % (len(rx), len(rk)))
del rx, rc, inv
order = np.argsort(rk)
rk, acc = rk[order], acc[order]

for name, path in CASES.items():
    try:
        xyz, rgb = load(path)
    except Exception as e:
        print("%-22s 读取失败 %s" % (name, e))
        continue
    k = key(xyz)
    pos = np.searchsorted(rk, k)
    pos = np.clip(pos, 0, len(rk) - 1)
    hit = rk[pos] == k
    if not hit.any():
        print("%-22s 无可比体素" % name)
        continue
    d = np.abs(rgb[hit].astype(np.int16) - acc[pos[hit]]).mean(1)
    print("%-22s 点 %d, 命中 %.1f%%, 色差 MAE %.2f, p95 %.0f, >30 %.2f%%"
          % (name, len(xyz), 100.0 * hit.mean(), d.mean(),
             np.percentile(d, 95), 100.0 * (d > 30).mean()))
    del xyz, rgb, k, pos, hit, d
