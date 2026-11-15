#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中段误差归因（第三步）：检验"近距误差 ∝ 1/z"这条时间对齐残差的特征。

前两步结论：
  1) LIO-VIO 位姿分歧全程 p90 ≤22mm / 0.46°，无中段抬升 → 排除位姿分歧；
  2) （段 × range）粗表显示 0-6m 段间 6.99→15.68 剧烈变化，6-15m 基本持平。
     LIO 段速度/角速度与近距 MAE 相关仅 0.50/0.60（6 个点，证据偏弱）。

本步判定标准：若误差来自"帧时间对齐残差 Δt"，则同一段内像素位移 ∝ 1/z，
色差应随距离近似 1/z 衰减（0-3m 到 10-15m 应差数倍）。
若同一段内近/远色差接近平坦，则不是 Δt 型误差，应转向曝光/遮挡/视角因素。

用法: python3 _probe_seg_range_law.py
"""
import numpy as np
from scipy.spatial import cKDTree

BAG = "/home/ywj/slam_data/official_vio_20261114/bag"
REF = "/home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb_v2.pcd"
CASE = "/home/ywj/slam_data/replay_node_onrecord_nocap_20261115/super_livo_visual_a_colored.pcd"
QN = 600000
SEGS = 6

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_lio():
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/aft_mapped_lidar_to_init"]))
    typ = get_message("nav_msgs/msg/Odometry")
    P = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, typ)
        p = m.pose.pose.position
        P.append([p.x, p.y, p.z])
    return np.array(P)


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


traj = cKDTree(read_lio())

ref, nref = view(REF)
ref_xyz = np.stack([ref["x"], ref["y"], ref["z"]], 1).astype(np.float64)
ref_rgb = rgb_of(ref)
print("参考(官方 v2) %d 点，建 KDTree ..." % nref)
tree_ref = cKDTree(ref_xyz)
del ref, ref_xyz

case, nc = view(CASE)
rng = np.random.default_rng(0)
idx = np.sort(rng.choice(nc, size=min(QN, nc), replace=False))
seg_id = (idx.astype(np.float64) / nc * SEGS).astype(np.int64).clip(0, SEGS - 1)
pts = case[idx]
xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
rgb = rgb_of(pts)

dr, jr = tree_ref.query(xyz, k=1, workers=-1)
ok = (dr < 0.02) & (rgb.sum(1) > 0) & (ref_rgb[jr].sum(1) > 0)
err = np.full(len(xyz), np.nan)
err[ok] = np.abs(rgb[ok] - ref_rgb[jr][ok]).mean(1)
dtr, _ = traj.query(xyz, k=1, workers=-1)

bins = [(0, 3), (3, 6), (6, 10), (10, 15), (15, 25)]
print("\n段 × 细 range 交叉表：MAE(p95) / d<2mm-MAE")
head = "%-4s" % "段"
for lo, hi in bins:
    head += "%18s" % ("%d-%dm" % (lo, hi))
print(head)
for s in range(SEGS):
    ms = seg_id == s
    row = "%-4d" % s
    for lo, hi in bins:
        m = ms & (dtr >= lo) & (dtr < hi) & ok
        if np.count_nonzero(m) < 200:
            row += "%18s" % "-"
            continue
        m2 = m & (dr < 0.002)
        e = err[m]
        row += "%18s" % ("%.1f(%.0f)/%.1f" % (
            np.nanmean(e), np.percentile(e, 95),
            np.nanmean(err[m2]) if m2.any() else float("nan")))
    print(row)

print("\n段内 1/z 检验：MAE(0-3m) / MAE(10-15m)  （Δt 型误差应 >>1，~3-5x）")
for s in range(SEGS):
    ms = seg_id == s
    a = ms & (dtr < 3) & ok
    b = ms & (dtr >= 10) & (dtr < 15) & ok
    if a.sum() < 100 or b.sum() < 100:
        print("  段%d 样本不足" % s)
        continue
    print("  段%d  n近=%6d n中=%6d  %.2f / %.2f = %.2fx" % (
        s, np.count_nonzero(a), np.count_nonzero(b),
        np.nanmean(err[a]), np.nanmean(err[b]),
        np.nanmean(err[a]) / np.nanmean(err[b])))

print("\n各段近距(0-6m) 误差分布与尾部:")
print("段      n     MAE   p50   p75   p90   p95  >30%  >60%")
for s in range(SEGS):
    m = (seg_id == s) & (dtr < 6) & ok
    if np.count_nonzero(m) < 200:
        continue
    e = err[m]
    print("%-4d %6d  %5.2f %5.0f %5.0f %5.0f %5.0f  %4.1f  %4.1f" % (
        s, len(e), np.nanmean(e), np.percentile(e, 50), np.percentile(e, 75),
        np.percentile(e, 90), np.percentile(e, 95),
        100.0 * (e > 30).mean(), 100.0 * (e > 60).mean()))
print("DONE_SEG_RANGE_LAW")
