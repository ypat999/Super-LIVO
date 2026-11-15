#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中段误差归因（第六步）：区分"场景难度（边缘/遮挡/暗光）"与"流水线差异"。

前几步已得到的事实：
  * 位姿分歧（LIO vs VIO）全程 p90 ≤22mm / 0.46°，无中段抬升 → 排除；
  * 段内 MAE(0-3m)/MAE(10-15m) ≈ 1.0 → 不是帧时间对齐残差（Δt 型）；
  * 30 桶时间剖面平滑升降（7.5 → 12 → 9），无陡峭起止边界；>30 坏点占比 2%→8.4%→2.7%；
  * 逐帧亮度 160 → 117 → 141（中段变暗），有符号色偏 +1.7 → +4.8 → -2.6。

本步在"官方参考"上算两个纯场景量，检查中段误差是否只是场景更难：
  contrast = 参考点 k=8 邻域颜色的标准差（越大越靠近边缘/纹理密集）；
  lum      = 参考点自身亮度。
若按 contrast / lum 分层后各段差异明显收敛，说明中段偏差来自场景，不是节点缺陷。

用法: python3 _probe_contrast.py
"""
import numpy as np
from scipy.spatial import cKDTree

REF = "/home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb_v2.pcd"
CASE = "/home/ywj/slam_data/replay_node_onrecord_nocap_20261115/super_livo_visual_a_colored.pcd"
QN = 700000
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

case, nc = view(CASE)
rng = np.random.default_rng(2)
idx = np.sort(rng.choice(nc, size=min(QN, nc), replace=False))
seg_id = (idx.astype(np.float64) / nc * SEGS).astype(np.int64).clip(0, SEGS - 1)
pts = case[idx]
xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
rgb = rgb_of(pts)

d, j = tree.query(xyz, k=9, workers=-1)
ok = (d[:, 0] < 0.02) & (rgb.sum(1) > 0) & (ref_rgb[j[:, 0]].sum(1) > 0)
# 邻域颜色标准差（不要自己：用 j[:,1:]）
nb = ref_rgb[j[:, 1:]].astype(np.float32)          # (N,8,3)
contrast = nb.reshape(len(nb), -1).std(1)
lum = ref_rgb[j[:, 0]].astype(np.float32).mean(1)
err = np.full(len(xyz), np.nan)
err[ok] = np.abs(rgb[ok] - ref_rgb[j[ok, 0]]).mean(1)

print("\n按邻域颜色对比度分层（MAE / 各段 MAE）：")
cb = [0, 3, 8, 15, 30, 60, 120]
head = "contrast     n    MAE" + "".join("%7s" % ("段%d" % s) for s in range(SEGS))
print(head)
for lo, hi in zip(cb[:-1], cb[1:]):
    m = ok & (contrast >= lo) & (contrast < hi)
    if np.count_nonzero(m) < 500:
        continue
    row = "[%3d,%3d) %6d  %5.2f" % (lo, hi, np.count_nonzero(m), np.nanmean(err[m]))
    for s in range(SEGS):
        ms = m & (seg_id == s)
        row += "%7s" % ("%.1f" % np.nanmean(err[ms]) if np.count_nonzero(ms) > 100 else "-")
    print(row)

print("\n按参考亮度分层（MAE / 各段 MAE）：")
lb = [0, 40, 70, 100, 130, 160, 200, 256]
head = "lum          n    MAE" + "".join("%7s" % ("段%d" % s) for s in range(SEGS))
print(head)
for lo, hi in zip(lb[:-1], lb[1:]):
    m = ok & (lum >= lo) & (lum < hi)
    if np.count_nonzero(m) < 500:
        continue
    row = "[%3d,%3d) %6d  %5.2f" % (lo, hi, np.count_nonzero(m), np.nanmean(err[m]))
    for s in range(SEGS):
        ms = m & (seg_id == s)
        row += "%7s" % ("%.1f" % np.nanmean(err[ms]) if np.count_nonzero(ms) > 100 else "-")
    print(row)

print("\n段对比度/亮度画像（中位）：")
print("段   contrast_p50  contrast_p90  lum_p50 (官方参考)")
for s in range(SEGS):
    m = ok & (seg_id == s)
    if np.count_nonzero(m) < 200:
        continue
    print("%-3d  %11.1f  %12.1f  %7.1f" % (
        s, np.median(contrast[m]), np.percentile(contrast[m], 90), np.median(lum[m])))
print("DONE_CONTRAST")
