#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中段误差归因：把"段"与"距离/视角"两个因素拆开。

前一步已排除 LIO-VIO 位姿分歧（各段 p90 平移均 ≤22mm、旋转 ≤0.46°，无中段抬升）。
这里检验另一个解释：中段是不是以"远距离、斜视角"为主，从而像素足迹变大、色差上升。

做法：对 PCD 采样点，算它到 LIO 轨迹的最近距离（range），
再按 range 分桶看 MAE；同时给出 (段 × range) 的交叉表，
若同一 range 下各段误差接近，则"段效应"其实是"距离效应"。

用法: python3 _probe_range_effect.py
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
    ts, P = [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, typ)
        ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        p = m.pose.pose.position
        P.append([p.x, p.y, p.z])
    return np.array(ts), np.array(P)


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


tl, Pl = read_lio()
print("LIO 轨迹 %d 个位置，span %.1fs" % (len(tl), tl[-1] - tl[0]))
traj = cKDTree(Pl)

ref, nref = view(REF)
ref_xyz = np.stack([ref["x"], ref["y"], ref["z"]], 1).astype(np.float64)
ref_rgb = rgb_of(ref)
print("参考(官方 v2) %d 点" % nref)
tree_ref = cKDTree(ref_xyz)
del ref, ref_xyz

case, nc = view(CASE)
rng = np.random.default_rng(0)
idx = np.sort(rng.choice(nc, size=min(QN, nc), replace=False))
seg_id = (idx.astype(np.float64) / nc * SEGS).astype(np.int64).clip(0, SEGS - 1)
pts = case[idx]
xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
rgb = rgb_of(pts)
print("采样 %d 点" % len(xyz))

dr, jr = tree_ref.query(xyz, k=1, workers=-1)
ok = (dr < 0.02) & (rgb.sum(1) > 0) & (ref_rgb[jr].sum(1) > 0)
err = np.full(len(xyz), np.nan)
err[ok] = np.abs(rgb[ok] - ref_rgb[jr][ok]).mean(1)
dtr, _ = traj.query(xyz, k=1, workers=-1)
print("同位率 %.1f%%" % (100.0 * ok.mean()))

edges = np.array([0, 3, 6, 10, 15, 25, 40, 80])
print("\n按到轨迹距离分桶:")
print("range(m)      n     同位%   MAE   MAE<2mm")
for lo, hi in zip(edges[:-1], edges[1:]):
    m = (dtr >= lo) & (dtr < hi) & ok
    if np.count_nonzero(m) < 500:
        continue
    m2 = m & (dr < 0.002)
    print("[%2d,%3d)  %7d  %5.1f  %5.2f  %5.2f" % (
        lo, hi, np.count_nonzero(m), 100.0 * m.sum() / np.count_nonzero((dtr >= lo) & (dtr < hi)),
        np.nanmean(err[m]), np.nanmean(err[m2]) if m2.any() else float("nan")))

print("\n段 × range 交叉表 (MAE / 同位%)：")
bins = [(0, 6), (6, 15), (15, 40)]
print("%-6s" % "段" + "".join("%16s" % ("%d-%dm" % b) for b in bins) + "   中位range")
for s in range(SEGS):
    row = "%d     " % s
    ms = seg_id == s
    for lo, hi in bins:
        m = ms & (dtr >= lo) & (dtr < hi) & ok
        tot = np.count_nonzero(ms & (dtr >= lo) & (dtr < hi))
        if np.count_nonzero(m) < 200:
            row += "%16s" % "-"
        else:
            row += "%16s" % ("%.2f / %.0f%%" % (np.nanmean(err[m]), 100.0 * m.sum() / tot))
    row += "   %6.2fm" % np.median(dtr[ms])
    print(row)
print("DONE_RANGE_EFFECT")
