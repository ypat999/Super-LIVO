#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Super-LIO 位姿 + 官方染色规则，离线生成彩色点云（单一变量对照用）。

目的
----
把「位姿来源」和「染色规则」解耦：
  * 位姿: Super-LIO /lio/odom（IMU 系，config/retail_lio.yaml 未设 odom_robo）
  * 点云: Super-LIO /lio/cloud_body_dense（去畸变后的 IMU 体系全分辨率扫描）
  * 染色: 完全复刻官方 FAST-LIVO2 LIVMapper::publish_frame_world
          仅 pf[2] > 0 + cam->isInFrame(bnd=3) + |p_w - p_wi| > blind_rgb_points，
          双线性取值；没有 z-buffer 遮挡、没有深度边缘剔除、没有质量阈值。
  * 图像: 原始 Retail_Street bag 的 /left_camera/image（畸变原图，与官方一致）

投影链（与 tools/singleframe_ab.py 已验证过的那套一致，官方发布 RGB 复现 MAE ~1）：
  p_c = Rci·Rwi(t_img)^T·(T_wi(t_cloud)·p_body - Pwi(t_img)) + Pci
  t_img = img_header + img_time_offset(0.1)
  Rci = Rcl·Rli^T, Pci = Pcl - Rcl·Rli^T·extT

输出点云只保留「被某张图成功着色的点」，与官方 pcl_wait_save 的语义一致。

用法:
  python3 15_superlio_official_color.py \
      --rec /home/ywj/slam_data/superlio_dense_20261114/rec \
      --out /home/ywj/slam_data/colored_superlio_official_20261114/superlio_official_rgb.pcd
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
import singleframe_ab as sf  # noqa: E402  复用已验证的投影/采样实现


def load_odom_clouds(path):
    """读 rec bag -> (odom 列表, body 云帧列表)。"""
    r = sf._reader(path)
    topics = {x.name: x.type for x in r.get_all_topics_and_types()}
    odom, frames = [], []
    while r.has_next():
        tup = r.read_next()
        topic = tup[0]
        msg = sf.deserialize_message(sf._payload(tup), sf._cls(topics[topic]))
        ts = sf._hdr(msg)
        if topic.endswith("odom"):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            odom.append((ts, (q.x, q.y, q.z, q.w), (p.x, p.y, p.z)))
        elif "cloud_body_dense" in topic:
            xyz, _, _ = sf.cloud_parts(msg)
            if len(xyz):
                frames.append((ts, xyz.astype(np.float32)))
    odom.sort(key=lambda a: a[0])
    frames.sort(key=lambda a: a[0])
    return odom, frames


def write_pcd(path, xyz, rgb):
    n = len(xyz)
    packed = (rgb[:, 0].astype(np.uint32) << 16) | \
             (rgb[:, 1].astype(np.uint32) << 8) | rgb[:, 2].astype(np.uint32)
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])
    arr = np.empty(n, dt)
    arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    arr["rgb"] = packed.view(np.float32)
    header = ("# .PCD v0.7 - Point Cloud Data file format\n"
              "VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
              "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n" % (n, n))
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        arr.tofile(f)


def frame_rms(xyz, rgb, vs=0.02):
    """单帧自身的 2cm 体素色 RMS（>=2 点），用于定位劣化发生在哪些帧。"""
    if len(xyz) < 2:
        return float("nan")
    key = np.floor(xyz / vs).astype(np.int64)
    key -= key.min(0)
    dims = key.max(0) + 1
    idx = (key[:, 0] * dims[1] + key[:, 1]) * dims[2] + key[:, 2]
    _, inv, cnt = np.unique(idx, return_inverse=True, return_counts=True)
    rms = np.zeros(len(cnt))
    for c in range(3):
        s = np.bincount(inv, weights=rgb[:, c].astype(np.float64), minlength=len(cnt))
        s2 = np.bincount(inv, weights=rgb[:, c].astype(np.float64) ** 2, minlength=len(cnt))
        rms += np.maximum(s2 / cnt - (s / cnt) ** 2, 0)
    sel = cnt >= 2
    if not sel.any():
        return float("nan")
    return float(np.sqrt(rms / 3.0)[sel].mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rec", default="/home/ywj/slam_data/superlio_dense_20261114/rec")
    ap.add_argument("--out", default="/home/ywj/slam_data/colored_superlio_official_20261114/"
                                     "superlio_official_rgb.pcd")
    ap.add_argument("--img-tol", type=float, default=0.05,
                    help="图像-点云 header 最近邻容差(s)；超出的帧不染色")
    ap.add_argument("--ext", choices=["official", "lidar"], default="official",
                    help="official=IMU->Camera(含杆臂, 与官方一致)；lidar=直接当 LiDAR 位姿用 Rcl/Pcl")
    ap.add_argument("--offset", type=float, default=sf.IMG_TIME_OFFSET,
                    help="取相机位姿用的图像时间偏移(s)，默认与官方 avia.yaml 一致 0.1")
    ap.add_argument("--max-frames", type=int, default=0, help="只处理前 N 帧（调试/扫描用）")
    ap.add_argument("--quick-metric", action="store_true", help="写出后顺便打印 2/5cm 体素色 RMS")
    ap.add_argument("--frame-csv", default="", help="可选：逐帧 2cm 色 RMS 诊断 CSV 路径")
    args = ap.parse_args()
    t0 = time.time()

    print("[1/4] 读 Super-LIO rec bag")
    odom, frames = load_odom_clouds(args.rec)
    print("   odom %d 条, body 云 %d 帧, stamp %.3f..%.3f"
          % (len(odom), len(frames), frames[0][0], frames[-1][0]))
    if not odom or not frames:
        sys.exit("rec bag 里没有 odom / cloud_body_dense")
    if args.max_frames > 0:
        frames = frames[:args.max_frames]

    track = sf.Track([a[0] for a in odom], [a[1] for a in odom], [a[2] for a in odom],
                     sf.GAP_NODE)
    imgs = sf.RawImages(frames[0][0], frames[-1][0])
    print("   原图 %d 张" % len(imgs.t))

    # 相机系外参（body->Camera）：
    #   official = Rci/Pci（IMU->Camera，含 IMU-LiDAR 杆臂），与官方 w2c 一致；
    #   lidar    = Rcl/Pcl（把 Super-LIO 的 IMU 位姿直接当 LiDAR 位姿，等同 A 节点的用法）
    T_ci = np.eye(4)
    if args.ext == "official":
        T_ci[:3, :3] = sf.RCI
        T_ci[:3, 3] = sf.PCI
    else:
        T_ci[:3, :3] = sf.RCL
        T_ci[:3, 3] = sf.PCL
    print("   ext=%s  offset=%.3f" % (args.ext, args.offset))

    print("[2/4] 逐帧复刻官方染色（无遮挡/无边缘/无质量阈值）")
    pw_all, rgb_all = [], []
    frame_rows = [] if args.frame_csv else None
    n_in = n_kept = n_skip_img = n_skip_pose = 0
    for k, (ts, p_body) in enumerate(frames):
        T_wc, ok_c = track.at(ts)
        if not ok_c:
            n_skip_pose += 1
            continue
        j = imgs.nearest(ts)
        hdr = imgs.t[j]
        if abs(hdr - ts) > args.img_tol:
            n_skip_img += 1
            continue
        T_wi, ok_i = track.at(hdr + args.offset)
        if not ok_i:
            n_skip_pose += 1
            continue

        # 点在世界系 -> 图像时刻相机系
        p_c = sf.xf(T_ci, sf.xf(np.linalg.inv(T_wi), sf.xf(T_wc, p_body)))
        u, v, okz = sf.official_uv(p_c)
        depth = np.linalg.norm(p_c, axis=1)
        sel = okz & sf.fov_official(u, v) & (depth > sf.BLIND_RGB)

        n_in += len(p_body)
        idx = np.where(sel)[0]
        if len(idx) == 0:
            continue
        _, img = imgs.get(j)
        rgb = sf.bilinear(img, u[idx], v[idx])
        ok_rgb = np.isfinite(rgb).all(axis=1)
        idx = idx[ok_rgb]
        if len(idx) == 0:
            continue
        p_w = sf.xf(T_wc, p_body[idx])
        rgb_u8 = np.clip(np.rint(rgb[ok_rgb]), 0, 255).astype(np.uint8)
        pw_all.append(p_w.astype(np.float32))
        rgb_all.append(rgb_u8)
        n_kept += len(idx)
        if frame_rows is not None:
            frame_rows.append((ts, len(p_body), len(idx), frame_rms(p_w, rgb_u8)))

        if (k + 1) % 100 == 0:
            print("   帧 %d/%d  已着色 %.1fM 点  用时 %.1fs"
                  % (k + 1, len(frames), n_kept / 1e6, time.time() - t0))

    print("   输入点 %d, 着色保留 %d (%.1f%%), 跳过: 无图 %d 帧 / 无位姿 %d 帧"
          % (n_in, n_kept, 100.0 * n_kept / max(n_in, 1), n_skip_img, n_skip_pose))
    if not pw_all:
        sys.exit("没有任何点被着色")

    print("[3/4] 写出 PCD")
    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    xyz = np.concatenate(pw_all, axis=0)
    rgb = np.concatenate(rgb_all, axis=0)
    del pw_all, rgb_all
    write_pcd(args.out, xyz, rgb)
    print("   %s  (%d 点, %.1f MB)"
          % (args.out, len(xyz), os.path.getsize(args.out) / 1e6))
    if frame_rows:
        with open(args.frame_csv, "w") as f:
            f.write("stamp,n_in,n_colored,rms2cm\n")
            for ts, ni, nc, rr in frame_rows:
                f.write("%.6f,%d,%d,%.3f\n" % (ts, ni, nc, rr))
        print("   逐帧诊断 -> %s" % args.frame_csv)
    if args.quick_metric:
        for vs in (0.02, 0.05):
            key = np.floor(xyz / vs).astype(np.int64)
            key -= key.min(0)
            dims = key.max(0) + 1
            idx = (key[:, 0] * dims[1] + key[:, 1]) * dims[2] + key[:, 2]
            _, inv, cnt = np.unique(idx, return_inverse=True, return_counts=True)
            rms = np.zeros(len(cnt))
            for c in range(3):
                s = np.bincount(inv, weights=rgb[:, c].astype(np.float64),
                                minlength=len(cnt))
                s2 = np.bincount(inv, weights=rgb[:, c].astype(np.float64) ** 2,
                                 minlength=len(cnt))
                rms += np.maximum(s2 / cnt - (s / cnt) ** 2, 0)
            print("   quick %.0fcm RMS(>=2pts) = %.2f"
                  % (vs * 100, np.sqrt(rms / 3.0)[cnt >= 2].mean()))
    print("[4/4] 完成，总耗时 %.1f s" % (time.time() - t0))


if __name__ == "__main__":
    main()
