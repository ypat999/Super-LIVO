#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""逐点比色已到噪声底（参考自身 5mm 邻点色差 10.49 > 我们 vs 参考 9.51）。
改用"空间一致性"：把点云体素化后取平均色再比，抑制逐点噪声、保留眼睛能看到的成片色差。

同时给出对照底噪：官方点云 vs 官方参考（两份都是官方产物），
若我们的体素误差与"官方 vs 官方"同量级，则不存在真实质量差距。

用法: python3 _probe_voxel_color_gap.py
"""
import numpy as np
import os
from scipy.spatial import cKDTree

BAG = "/home/ywj/slam_data/official_vio_20261114/bag"
REF = "/home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb_v2.pcd"
OURS = os.environ.get(
    "CASE", "/home/ywj/slam_data/replay_node_onrecord_nocap_20261115/super_livo_visual_a_colored.pcd")
OFFI = os.environ.get(
    "OFFI", "/home/ywj/slam_data/colored_superlio_official_20261114/superlio_official_rgb.pcd")
VOX = 0.05
MAXR = 5.0
QN = 400000

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


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
    return np.stack([(v >> 16) & 255, (v >> 8) & 255, v & 255], 1).astype(np.float64)


def voxelize(path, idx=None):
    arr, n = view(path)
    if idx is not None:
        arr = arr[idx]
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float64)
    rgb = rgb_of(arr)
    q = np.floor(xyz / VOX).astype(np.int64) + (1 << 20)
    key = (q[:, 0] << 42) | (q[:, 1] << 21) | q[:, 2]
    del q
    _, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    del key
    cen = np.stack([np.bincount(inv, weights=xyz[:, i], minlength=len(cnt))
                    for i in range(3)], 1) / cnt[:, None]
    col = np.stack([np.bincount(inv, weights=rgb[:, i], minlength=len(cnt))
                    for i in range(3)], 1) / cnt[:, None]
    print("  %s: %d 点 -> %d 体素(%.0fcm)" % (path.split("/")[-1], n, len(cnt), VOX * 100))
    return cen, col, cnt


def read_traj():
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/aft_mapped_lidar_to_init"]))
    typ = get_message("nav_msgs/msg/Odometry")
    P = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, typ)
        P.append([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
    return np.array(P)


def compare(name, cen, col, nv, rcen, rcol, ttree):
    rng = np.random.default_rng(0)
    sel = np.sort(rng.choice(len(cen), size=min(QN, len(cen)), replace=False))
    c, v, nvv = cen[sel], col[sel], nv[sel]
    tr = cKDTree(rcen)
    d, j = tr.query(c, k=1, workers=-1)
    ok = d < MAXR
    e = np.abs(v[ok] - rcol[j[ok]]).mean(1)
    print("\n== %s ==  体素命中 %.1f%%  匹配距离中位 %.1f mm" % (
        name, 100.0 * ok.mean(), 1000 * np.median(d[ok])))
    print("  体素色差 MAE %.2f  p50 %.2f  p75 %.2f  p90 %.2f  p95 %.2f  >20 占比 %.1f%%" % (
        e.mean(), *[np.percentile(e, p) for p in (50, 75, 90, 95)],
        100.0 * (e > 20).mean()))
    print("  分段（按累积顺序 = 时间顺序 6 等分）:")
    seg = np.clip((np.arange(len(c))[ok].astype(float) / len(c) * 6).astype(int), 0, 5)
    for s in range(6):
        m = seg == s
        if m.sum() > 100:
            print("    段%d  %6.1f%%   MAE %6.2f   p90 %6.2f" % (
                s, 100.0 * m.mean(), e[m].mean(), np.percentile(e[m], 90)))
    print("  按体素点数分层（点数=该 5cm 体素的历史观测点数）:")
    nn = nvv[ok]
    for lo, hi in [(1, 1), (2, 2), (3, 5), (6, 10), (11, 30), (31, 10 ** 9)]:
        m = (nn >= lo) & (nn <= hi)
        if m.sum() < 50:
            continue
        print("    %3d-%-3s  %6.1f%%   MAE %6.2f  p90 %6.2f  >20 %5.1f%%" % (
            lo, hi if hi < 10 ** 9 else "+", 100.0 * m.mean(),
            e[m].mean(), np.percentile(e[m], 90), 100.0 * (e[m] > 20).mean()))
    # 最差体素（眼睛最容易看出来的成片错误）
    ki = np.where(ok)[0]
    order = np.argsort(-e)[:12]
    print("  最差 12 个体素（x, y, z | 距轨迹 | 色差 | 点数）:")
    for i in order:
        k = ki[i]
        print("    %7.2f %7.2f %6.2f | %5.1fm | %6.1f | %d" % (
            c[k, 0], c[k, 1], c[k, 2], ttree.query(c[k:k + 1])[0][0], e[i], nvv[k]))
    return e


def pair_stat(name, ca, va, cb, vb):
    """两个体素云互比，只打印核心统计（用于底噪/密度对照）。"""
    tr = cKDTree(cb)
    d, j = tr.query(ca, k=1, workers=-1)
    ok = d < MAXR
    e = np.abs(va[ok] - vb[j[ok]]).mean(1)
    print("  %-28s 体素%d  MAE %.2f  p50 %.2f  p90 %.2f  >20 %.1f%%" % (
        name, len(ca), e.mean(), np.percentile(e, 50),
        np.percentile(e, 90), 100.0 * (e > 20).mean()))
    return e


def main():
    print("体素化参考 ...")
    rcen, rcol, rcnt = voxelize(REF)
    ttree = cKDTree(read_traj())

    # ---- 指标底噪：把同一份参考点云随机劈成两半，各自体素化后互比 ----
    # 注意：劈半后每个体素的平均点数减半，噪声被放大 √2 倍。
    print("\n参考自劈半对照（指标底噪）...")
    _, nref_tot = view(REF)
    rng = np.random.default_rng(1)
    perm = rng.permutation(nref_tot)
    ha, hb = np.sort(perm[: nref_tot // 2]), np.sort(perm[nref_tot // 2:])
    ca, va, na = voxelize(REF, ha)
    cb, vb, nb = voxelize(REF, hb)
    pair_stat("参考半A vs 参考半B", ca, va, cb, vb)
    del ca, va, cb, vb

    print("\n体素化本节点 ...")
    cen, col, cnt = voxelize(OURS)
    compare("本节点(nocap) vs 官方参考", cen, col, cnt, rcen, rcol, ttree)

    # ---- 密度对照：把本节点也降到同样半密度，隔离"点数"带来的指标噪声 ----
    _, nours = view(OURS)
    io = np.sort(np.random.default_rng(2).choice(nours, size=nours // 2, replace=False))
    co, vo, no = voxelize(OURS, io)
    pair_stat("本节点(半密度) vs 全参考", co, vo, rcen, rcol)
    del cen, col, cnt, co, vo

    print("\n体素化官方染色点云 ...")
    if OFFI and os.path.exists(OFFI):
        cen2, col2, cnt2 = voxelize(OFFI)
        compare("官方点云 vs 官方参考（对照底噪）", cen2, col2, cnt2, rcen, rcol, ttree)
    else:
        print("  (跳过)")
    print("\nDONE_VOXEL_GAP")


if __name__ == "__main__":
    main()
