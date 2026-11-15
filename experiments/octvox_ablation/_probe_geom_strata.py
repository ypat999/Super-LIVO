#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把彩色点云的比色残差按"几何条件"分层，判断剩余误差还剩多少可压。

背景（已确认）：
  * LIO/VIO 位姿分歧 p90 ≤20mm/0.39°，无中段抬升；
  * 投影无系统性像素偏移（±12px 网格搜索，最优位移恒为 (0,0)，收益 0.13）；
  * 单帧着色 MAE 10.63、时间中位 9.94 —— 误差在单帧取样阶段就已存在。

本脚本只用已落盘的彩色 PCD，不重放、不重编译：
  1) 用累积点云自身 kNN 协方差求法向：
       - 平面厚度 sqrt(λ0)（米）→ 越厚越像"边缘/细结构/混合像素"；
  2) 入射角 = 视线（点 - 最近 LIO 轨迹点）与法向的夹角 → 掠射面像素足迹被拉长；
  3) 与官方参考色对齐后，按上述两个量分层看 MAE / p95 / >30% 坏点率，
     并给出"各层对总误差的贡献占比"，回答"压哪里最划算"。

用法: python3 _probe_geom_strata.py
"""
import numpy as np
from scipy.spatial import cKDTree

BAG = "/home/ywj/slam_data/official_vio_20261114/bag"
REF = "/home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb_v2.pcd"
CASE = "/home/ywj/slam_data/replay_node_onrecord_nocap_20261115/super_livo_visual_a_colored.pcd"
QN = 300000
KNN = 16

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
    dt = np.dtype({"names": f, "formats": ["<f4"] * len(f),
                   "offsets": [4 * i for i in range(len(f))], "itemsize": 4 * len(f)})
    return np.memmap(path, dtype=dt, mode="r", offset=off, shape=(n,)), n


def rgb_of(arr):
    key = "rgb" if "rgb" in arr.dtype.names else "rgba"
    v = np.asarray(arr[key]).view(np.uint32)
    return np.stack([(v >> 16) & 255, (v >> 8) & 255, v & 255], 1).astype(np.int16)


def table(name, bins, x, err, maxtot):
    print("\n== %s ==" % name)
    print("  %-16s %7s %8s %8s %8s %9s" % ("区间", "点数%", "MAE", "p95", ">30%", "误差贡献%"))
    idx = np.digitize(x, bins)
    for i in range(len(bins) + 1):
        m = idx == i
        if m.sum() < 200:
            continue
        e = err[m]
        lo = "-inf" if i == 0 else "%.3f" % bins[i - 1]
        hi = "+inf" if i >= len(bins) else "%.3f" % bins[i]
        share = 100.0 * e.sum() / maxtot
        print("  [%7s,%7s] %6.1f%% %8.2f %8.2f %7.1f%% %8.1f%%" % (
            lo, hi, 100.0 * m.mean(), e.mean(), np.percentile(e, 95),
            100.0 * (e > 76.5).mean(), share))


def main():
    # ---- 本次彩色点云 ----
    case, nc = view(CASE)
    print("本节点彩色 PCD %d 点" % nc)
    rng = np.random.default_rng(0)
    sidx = np.sort(rng.choice(nc, size=min(QN, nc), replace=False))
    ours_rgb = rgb_of(case[sidx])
    xyz_all = np.stack([case["x"], case["y"], case["z"]], 1).astype(np.float64)
    pts = xyz_all[sidx]

    # ---- 法向 / 平面厚度（用累积点云自身 kNN） ----
    print("建自身 KDTree（%.1fM 点）..." % (nc / 1e6))
    tree = cKDTree(xyz_all)
    _, nb = tree.query(pts, k=KNN, workers=-1)
    nbxyz = xyz_all[nb]
    del tree
    nbxyz -= nbxyz.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nbxyz, nbxyz) / float(KNN)
    del nbxyz, nb
    w, v = np.linalg.eigh(cov)
    del cov
    thick = np.sqrt(np.clip(w[:, 0], 0.0, None))  # 平面厚度（m）
    lam1 = np.clip(w[:, 1], 1e-12, None)
    edgey = np.sqrt(w[:, 0] / lam1)              # 0=平面, 大=线/边缘
    nrm = np.array(v[:, :, 0])                   # 最小特征向量 = 法向
    del w, v

    # ---- 入射角（视线取最近 LIO 轨迹点，与相机仅差 ~0.1m） ----
    print("读 LIO 轨迹 ...")
    traj = read_lio()
    ttree = cKDTree(traj)
    rng_dist, ti = ttree.query(pts, k=1, workers=-1)
    d = pts - traj[ti]
    d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
    cos_inc = np.abs(np.einsum("ni,ni->n", nrm, d))
    inc = np.degrees(np.arccos(np.clip(cos_inc, 0.0, 1.0)))
    del nrm, d

    # ---- 与官方参考色对齐 ----
    print("建参考 KDTree ...")
    ref, nref = view(REF)
    ref_xyz = np.stack([ref["x"], ref["y"], ref["z"]], 1).astype(np.float64)
    ref_rgb = rgb_of(ref)
    rtree = cKDTree(ref_xyz)
    dd, jr = rtree.query(pts, k=2, workers=-1)
    print("参考 %d 点，d<5mm 命中 %.1f%%" % (nref, 100.0 * (dd[:, 0] < 0.005).mean()))

    # ---- 关键对照：参考点云自身的"5mm 内颜色噪声" ----
    # 若我们的 MAE 与这个自噪声同量级，说明比色指标已到对应误差底噪，
    # 残差主要来自"最近邻配不准"而不是我们的着色出错。
    near = dd[:, 0] < 0.005
    dd1, jr1 = dd[near, 0], jr[near, 0]
    self_pair = np.abs(ref_rgb[jr1].astype(np.float32) -
                       ref_rgb[jr[near, 1]].astype(np.float32)).mean(1)
    self_d = dd[near, 1]
    print("参考自噪声(1st vs 2nd NN): MAE %.2f  p95 %.2f  |  2nd NN 距离中位 %.1f mm" % (
        self_pair.mean(), np.percentile(self_pair, 95), 1000.0 * np.median(self_d)))

    err = np.abs(ours_rgb[near].astype(np.float32) - ref_rgb[jr1].astype(np.float32)).mean(1)
    thick, edgey, inc, rr = thick[near], edgey[near], inc[near], rng_dist[near]
    tot = err.sum()
    print("总体 MAE %.2f  p95 %.2f  >30%% %.1f%%  点数 %d" % (
        err.mean(), np.percentile(err, 95), 100.0 * (err > 76.5).mean(), len(err)))
    print("分位: p50 %.2f / p75 %.2f / p90 %.2f / p95 %.2f  （自噪声 p50 %.2f / p90 %.2f / p95 %.2f）" % (
        np.percentile(err, 50), np.percentile(err, 75), np.percentile(err, 90),
        np.percentile(err, 95),
        np.percentile(self_pair, 50), np.percentile(self_pair, 90), np.percentile(self_pair, 95)))

    table("最近邻距离 [mm]（对应误差，检验是否被对应噪声主导）",
          [0.5e-3, 1.0e-3, 2.0e-3, 3.5e-3], dd1, err, tot)
    table("平面厚度 sqrt(λ0) [m]（大=边缘/细结构/混合像素）",
          [0.003, 0.006, 0.010, 0.020, 0.040, 0.080], thick, err, tot)
    table("边缘度 sqrt(λ0/λ1)（大=线状/角点）",
          [0.10, 0.20, 0.35, 0.60, 1.00], edgey, err, tot)
    table("入射角 [deg]（大=掠射，像素足迹被拉长）",
          [15.0, 30.0, 45.0, 60.0, 75.0], inc, err, tot)
    table("距离 [m]（对照项）",
          [3.0, 6.0, 10.0, 15.0, 25.0], rr, err, tot)
    print("\nDONE_GEOM_STRATA")


if __name__ == "__main__":
    main()
