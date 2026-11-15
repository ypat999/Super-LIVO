#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中段误差归因（第五步）：误差随"时间"的细粒度剖面 + 有符号色偏。

判定依据：
  * 若某段有系统性有符号色偏（三通道同向）→ 曝光/增益类问题；
  * 若误差是零均值但重尾、且**起止边界陡峭** → 该窗口的帧关联错了（图像配对错帧）；
  * 若缓慢平滑地升降 → 场景/光照渐变。

时间轴用 PCD 落盘顺序（节点按时间 append），30 个细桶。

用法: python3 _probe_time_profile.py
"""
import numpy as np
from scipy.spatial import cKDTree

REF = "/home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb_v2.pcd"
CASE = "/home/ywj/slam_data/replay_node_onrecord_nocap_20261115/super_livo_visual_a_colored.pcd"
QN = 900000
SEGS = 6
NBIN = 30

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = "/home/ywj/slam_data/official_vio_20261114/bag"


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
print("参考 %d 点，建 KDTree ..." % nref)
tree_ref = cKDTree(ref_xyz)
del ref, ref_xyz

case, nc = view(CASE)
rng = np.random.default_rng(1)
idx = np.sort(rng.choice(nc, size=min(QN, nc), replace=False))
pts = case[idx]
xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
rgb = rgb_of(pts)

dr, jr = tree_ref.query(xyz, k=1, workers=-1)
ok = (dr < 0.02) & (rgb.sum(1) > 0) & (ref_rgb[jr].sum(1) > 0)
diff = np.zeros((len(xyz), 3), np.float32)
diff[ok] = rgb[ok].astype(np.float32) - ref_rgb[jr][ok].astype(np.float32)
err = np.abs(diff).mean(1)
err[~ok] = np.nan
dtr, _ = traj.query(xyz, k=1, workers=-1)
tsec = idx / nc * 135.47  # 原始包时长

print("\n[30 细桶] t(s)  n   MAE  >30%   bias(R,G,B)   近距MAE")
edges = np.linspace(0, 135.47, NBIN + 1)
for b in range(NBIN):
    m = ok & (tsec >= edges[b]) & (tsec < edges[b + 1])
    if np.count_nonzero(m) < 300:
        continue
    e = err[m]
    bi = diff[m].mean(0)
    mn = m & (dtr < 6)
    print("%5.1f %6d %5.2f %5.1f%%  %+5.1f %+5.1f %+5.1f   %5.2f" % (
        edges[b], np.count_nonzero(m), np.nanmean(e), 100.0 * (e > 30).mean(),
        bi[0], bi[1], bi[2],
        np.nanmean(err[mn]) if mn.any() else float("nan")))

print("\n[6 段] 有符号偏差与离散度分解 (d<5mm 同位点):")
print("段    n     bias(R,G,B)        |bias|   rms(err)   MAE  >30%")
for s in range(SEGS):
    m = ok & (tsec >= edges[s * NBIN // SEGS]) & (tsec < edges[(s + 1) * NBIN // SEGS])
    m &= dr < 0.005
    if np.count_nonzero(m) < 300:
        continue
    bi = diff[m].mean(0)
    print("%-3d %6d  %+5.1f %+5.1f %+5.1f  %6.2f  %7.2f  %5.2f  %4.1f%%" % (
        s, np.count_nonzero(m), bi[0], bi[1], bi[2], np.linalg.norm(bi),
        np.sqrt((diff[m] ** 2).mean()), np.nanmean(err[m]),
        100.0 * (err[m] > 30).mean()))
print("DONE_TIME_PROFILE")
