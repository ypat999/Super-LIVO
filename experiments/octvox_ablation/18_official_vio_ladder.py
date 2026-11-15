#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A 方案染色 -> 官方染色的逐级替换（阶梯实验，位姿全部固定为官方 VIO 最终位姿）。

数据来源：experiments/octvox_ablation/17_run_official_vio_record.sh 录的 bag
  /vio/odom_lidar           官方 handleVIO 中 processFrame 之后的 LiDAR 位姿（VIO 修正后）
  /aft_mapped_lidar_to_init 官方 handleLIO 的 LiDAR 位姿（官方染色的 p_w 由它得到）
  /cloud_registered_body    同戳去畸变全分辨率扫描（dense_map_en=true -> feats_undistort）
  /cloud_registered         官方自带彩色世界云（参考色，仅保留被官方着色的点）
图像：Retail_Street 原始 /left_camera/image（未去畸变，与官方一致）

阶梯（每一级只去掉 A 方案的一个环节）：
  R0   A 全量：整流图 + 针孔 + z-buffer 遮挡 + 3x3 深度边缘 + quality>=0.2 + 1cm x K6 时间中位色
  R1   R0 - 1cm x K6 时间中位数（改用本帧自己的采样色）
  R2   R1 - quality>=0.2
  R3   R2 - 3x3 深度边缘剔除
  R4   R3 - z-buffer 遮挡剔除
  R5   R4 - 预去畸变（换回原图 + 解析畸变）== 官方 publish_frame_world
  R0b  R0 但换成 A 节点真实配对（图像 header≈ts，相机位姿 header+0.1；官方是同一时刻 ts）

用法:
  python3 18_official_vio_ladder.py \
      --rec /home/ywj/slam_data/official_vio_20261114/bag \
      --out /home/ywj/slam_data/official_vio_20261114/ladder
"""

import argparse
import os
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
import singleframe_ab as sf  # noqa: E402

RUNGS = ["R0", "R1", "R2", "R3", "R4", "R5", "R0b"]
TITLES = {
    "R0": "R0  A全量(整流+遮挡+边缘+质量+1cm中位)",
    "R1": "R1  - 1cm中位色",
    "R2": "R2  - quality>=0.2",
    "R3": "R3  - 深度边缘剔除",
    "R4": "R4  - z-buffer遮挡",
    "R5": "R5  - 预去畸变(=官方)",
    "R0b": "R0b A全量 + A节点真实配对(图像h≈ts, 位姿h+0.1)",
}


# ----------------------------------------------------------------------------- 读包
def load_rec(path):
    r = sf._reader(path)
    tt = {x.name: x.type for x in r.get_all_topics_and_types()}
    vio, lio, body, colored = [], [], [], []
    while r.has_next():
        tup = r.read_next()
        topic = tup[0]
        m = sf.deserialize_message(sf._payload(tup), sf._cls(tt[topic]))
        ts = sf._hdr(m)
        if topic == "/vio/odom_lidar":
            p, q = m.pose.pose.position, m.pose.pose.orientation
            vio.append((ts, (q.x, q.y, q.z, q.w), (p.x, p.y, p.z)))
        elif topic == "/aft_mapped_lidar_to_init":
            p, q = m.pose.pose.position, m.pose.pose.orientation
            lio.append((ts, (q.x, q.y, q.z, q.w), (p.x, p.y, p.z)))
        elif topic == "/cloud_registered_body":
            xyz, _, _ = sf.cloud_parts(m)
            if len(xyz):
                body.append((ts, xyz))
        elif topic == "/cloud_registered":
            xyz, rgb, _ = sf.cloud_parts(m)
            if len(xyz) and rgb is not None:
                colored.append((ts, xyz, rgb))
    r.close() if hasattr(r, "close") else None
    vio.sort(key=lambda a: a[0])
    lio.sort(key=lambda a: a[0])
    body.sort(key=lambda a: a[0])
    return vio, lio, body, colored


def _mk_track(rows):
    return sf.Track([a[0] for a in rows], [a[1] for a in rows],
                    [a[2] for a in rows], sf.GAP_NODE)


def _exact_lookup(rows):
    return {round(a[0], 6): a for a in rows}


# ----------------------------------------------------------------------------- 写盘
class PcdStream(object):
    def __init__(self, bin_path):
        self.f = open(bin_path, "wb")
        self.n = 0

    def add(self, xyz, rgb):
        n = len(xyz)
        if n == 0:
            return
        packed = (rgb[:, 0].astype(np.uint32) << 16) | \
                 (rgb[:, 1].astype(np.uint32) << 8) | rgb[:, 2].astype(np.uint32)
        dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])
        arr = np.empty(n, dt)
        arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        arr["rgb"] = packed.view(np.float32)
        arr.tofile(self.f)
        self.n += n

    def finish(self, pcd_path):
        self.f.close()
        header = ("# .PCD v0.7 - Point Cloud Data file format\n"
                  "VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
                  "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
                  % (self.n, self.n))
        with open(pcd_path, "wb") as out:
            out.write(header.encode("ascii"))
            with open(self.f.name, "rb") as src:
                while True:
                    buf = src.read(1 << 24)
                    if not buf:
                        break
                    out.write(buf)
        os.remove(self.f.name)
        return self.n


# ----------------------------------------------------------------------------- 指标
def voxel_rms(xyz, rgb, vs):
    if len(xyz) < 2:
        return float("nan")
    key = np.floor(xyz / vs).astype(np.int64)
    key -= key.min(0)
    dims = key.max(0) + 1
    idx = (key[:, 0] * dims[1] + key[:, 1]) * dims[2] + key[:, 2]
    _, inv, cnt = np.unique(idx, return_inverse=True, return_counts=True)
    acc = np.zeros(len(cnt))
    for c in range(3):
        w = rgb[:, c].astype(np.float64)
        s = np.bincount(inv, weights=w, minlength=len(cnt))
        s2 = np.bincount(inv, weights=w * w, minlength=len(cnt))
        acc += np.maximum(s2 / cnt - (s / cnt) ** 2, 0.0)
    sel = cnt >= 2
    if not sel.any():
        return float("nan")
    return float(np.sqrt(acc / 3.0)[sel].mean())


def median_colors(pw, rgb, vs=0.01, K=6):
    """复刻 VoxelColorMedian：1cm 体素、最近 K 次观测、逐通道时间中位数。"""
    n = len(pw)
    p32 = pw.astype(np.float32)
    kx = np.floor(p32[:, 0] * (1.0 / vs)).astype(np.int64)
    ky = np.floor(p32[:, 1] * (1.0 / vs)).astype(np.int64)
    kz = np.floor(p32[:, 2] * (1.0 / vs)).astype(np.int64)
    off = 1 << 20
    key = ((kx + off) << 42) | ((ky + off) << 21) | (kz + off)
    order = np.argsort(key, kind="stable")
    ks = key[order]
    new = np.empty(n, bool)
    new[0] = True
    np.not_equal(ks[1:], ks[:-1], out=new[1:])
    starts = np.flatnonzero(new)
    gid_sorted = np.cumsum(new) - 1
    counts = np.diff(np.r_[starts, n])
    g = len(starts)
    pos_in = np.arange(n) - np.repeat(starts, counts)
    rank = np.repeat(counts, counts) - 1 - pos_in
    keep = rank < K
    buf = np.zeros((g, K, 3), np.uint8)
    buf[gid_sorted[keep], rank[keep]] = rgb[order][keep]
    cnt = np.minimum(counts, K)
    vals = buf.astype(np.int16)
    vals[np.arange(K)[None, :] >= cnt[:, None]] = 32767
    vals.sort(axis=1)
    lo = ((cnt - 1) // 2)[:, None, None].repeat(3, 2)
    hi = (cnt // 2)[:, None, None].repeat(3, 2)
    med = (np.take_along_axis(vals, lo, axis=1) +
           np.take_along_axis(vals, hi, axis=1)) * 0.5
    med = med[:, 0, :]
    gid_of_point = np.empty(n, np.int64)
    gid_of_point[order] = gid_sorted
    return np.clip(med[gid_of_point] + 0.5, 0, 255).astype(np.uint8)


# ----------------------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rec", default="/home/ywj/slam_data/official_vio_20261114/bag")
    ap.add_argument("--out", default="/home/ywj/slam_data/official_vio_20261114/ladder")
    ap.add_argument("--img-tol", type=float, default=0.05)
    ap.add_argument("--subsample", type=int, default=10,
                    help="内部一致性 RMS 指标用的抽帧步长")
    ap.add_argument("--no-offset-rung", action="store_true", help="不跑 R0b")
    args = ap.parse_args()
    t0 = time.time()
    os.makedirs(args.out, exist_ok=True)

    print("[1/4] 读包")
    vio, lio, body, colored = load_rec(args.rec)
    print("   VIO位姿 %d / LIO位姿 %d / body云 %d / 官方彩色云 %d"
          % (len(vio), len(lio), len(body), len(colored)))
    if not vio or not body:
        sys.exit("bag 缺少 /vio/odom_lidar 或 /cloud_registered_body")
    trk_vio = _mk_track(vio)
    trk_lio = _mk_track(lio)
    vio_by_ts = _exact_lookup(vio)
    # 官方彩色云与 VIO 位姿在同一回调中先后发布 -> 按顺序一一对应
    ref_by_ts = {}
    if len(colored) == len(vio):
        for (ts_v, _, _), (_, cxyz, crgb) in zip(vio, colored):
            ref_by_ts[round(ts_v, 6)] = (cxyz, crgb)
        print("   官方彩色云按 VIO 顺序配对成功")
    else:
        print("   [warn] 官方彩色云 %d 与 VIO 位姿 %d 数量不一致，退化为按几何最近帧配对"
              % (len(colored), len(vio)))
        ref_by_ts = None

    imgs = sf.RawImages(body[0][0], body[-1][0])
    print("   原图 %d 张" % len(imgs.t))

    print("[2/4] 逐帧计算 A 方案各环节 / 官方投影")
    data = []
    n_in = 0
    n_skip = 0
    n_ref_ok = 0
    for k, (ts, p_body) in enumerate(body):
        T_lio, ok2 = trk_lio.at(ts)
        if not ok2:
            n_skip += 1
            continue
        # 官方同步：img_time_buffer 存 header+img_time_offset(0.1)，
        # img_capture_time = header+0.1 就是本次 LIO/VIO 的时刻（= 点云戳 ts）；
        # 因此官方实际用的图像是 header ≈ ts-0.1 的那一张，相机位姿也取 ts 时刻。
        j = imgs.nearest(ts - sf.IMG_TIME_OFFSET)
        hdr = imgs.t[j]
        t_cam = hdr + sf.IMG_TIME_OFFSET
        if abs(t_cam - ts) > args.img_tol:
            n_skip += 1
            continue
        T_vio, ok1 = trk_vio.at(t_cam)
        if not ok1:
            n_skip += 1
            continue
        n_in += len(p_body)
        p_w = sf.xf(T_lio, p_body)                     # 官方 p_w（handleLIO 位姿）
        p_l = sf.xf(np.linalg.inv(T_vio), p_w)         # VIO 相机位姿下的 LiDAR 系
        p_c = p_l @ sf.RCL.T + sf.PCL[None, :]
        u, v, okz = sf.pinhole(p_c)
        in_fov = sf.fov_custom(u, v)
        zb = sf.zbuffer(u, v, p_c[:, 2], in_fov & okz)
        vis, edge = sf.vis_edge(zb, u, v, p_c[:, 2])
        z = np.where(okz, p_c[:, 2], 1e9)
        margin = np.minimum(np.minimum(u, sf.CW - u), np.minimum(v, sf.CH - v))
        quality = np.clip(margin / 20.0, 0.0, 1.0) * np.minimum(1.0, 1.0 / (1.0 + z / 50.0))
        m4 = in_fov & okz
        m3 = m4 & vis
        m2 = m3 & ~edge
        m0 = m2 & (quality >= 0.2)

        ud, vd, okz_o = sf.official_uv(p_c)
        pf_norm = np.linalg.norm(p_c, axis=1)
        m_off = sf.fov_official(ud, vd) & okz_o & (pf_norm > sf.BLIND_RGB)

        _, im = imgs.get(j)
        rimg = sf.rectify_u8(im)
        rgb_a = np.zeros((len(p_body), 3), np.uint8)
        ok = m4
        rgb_a[ok] = np.clip(np.rint(sf.bilinear(rimg, u[ok], v[ok])), 0, 255).astype(np.uint8)
        rgb_o = np.zeros((len(p_body), 3), np.uint8)
        rgb_o[m_off] = np.clip(np.rint(sf.bilinear(im, ud[m_off], vd[m_off])),
                               0, 255).astype(np.uint8)

        rec = {"pw": p_w.astype(np.float32), "rgbA": rgb_a, "rgbO": rgb_o,
               "m0": m0, "m2": m2, "m3": m3, "m4": m4, "moff": m_off}
        if not args.no_offset_rung:
            # 对照：A 节点真实配对——图像取离点云戳最近的那张（header≈ts），
            # 相机位姿取图像采集时刻 header+0.1（≈ts+0.1，官方是同一时刻 ts）。
            j2 = imgs.nearest(ts)
            _, im2 = imgs.get(j2)
            rimg2 = sf.rectify_u8(im2)
            T_img, ok3 = trk_vio.at(imgs.t[j2] + sf.IMG_TIME_OFFSET)
            if ok3:
                p_l2 = sf.xf(np.linalg.inv(T_img), p_w)
                p_c2 = p_l2 @ sf.RCL.T + sf.PCL[None, :]
                u2, v2, okz2 = sf.pinhole(p_c2)
                f2 = sf.fov_custom(u2, v2)
                zb2 = sf.zbuffer(u2, v2, p_c2[:, 2], f2 & okz2)
                vis2, edge2 = sf.vis_edge(zb2, u2, v2, p_c2[:, 2])
                z2 = np.where(okz2, p_c2[:, 2], 1e9)
                m2b = np.minimum(np.minimum(u2, sf.CW - u2), np.minimum(v2, sf.CH - v2))
                q2 = np.clip(m2b / 20.0, 0.0, 1.0) * np.minimum(1.0, 1.0 / (1.0 + z2 / 50.0))
                rec["m0b"] = f2 & okz2 & vis2 & ~edge2 & (q2 >= 0.2)
                rgb_b = np.zeros((len(p_body), 3), np.uint8)
                sel = f2 & okz2
                rgb_b[sel] = np.clip(np.rint(sf.bilinear(rimg2, u2[sel], v2[sel])),
                                     0, 255).astype(np.uint8)
                rec["rgbB"] = rgb_b
            else:
                rec["m0b"] = np.zeros(len(p_body), bool)
                rec["rgbB"] = np.zeros((len(p_body), 3), np.uint8)

        # 参考色：官方 /cloud_registered（同戳）
        cv = vio_by_ts.get(round(ts, 6))
        ref_rgb = None
        if ref_by_ts is not None and cv is not None:
            ent = ref_by_ts.get(round(cv[0], 6))
            if ent is not None:
                cxyz, crgb = ent
                if len(cxyz) == int(m_off.sum()):
                    ref_rgb = np.zeros((len(p_body), 3), np.uint8)
                    ref_rgb[m_off] = crgb
                    n_ref_ok += 1
                else:
                    tree = cKDTree(p_w.astype(np.float64))
                    dd, ii = tree.query(cxyz)
                    if np.median(dd) < 1e-3:
                        ref_rgb = np.zeros((len(p_body), 3), np.uint8)
                        ref_rgb[ii[dd < 1e-3]] = crgb[dd < 1e-3]
                        n_ref_ok += 1
        rec["ref"] = ref_rgb
        data.append(rec)
        if (k + 1) % 200 == 0:
            print("   %d/%d 帧 %.1fs" % (k + 1, len(body), time.time() - t0))

    print("   有效帧 %d，跳过 %d，输入点 %d，官方参考可用帧 %d"
          % (len(data), n_skip, n_in, n_ref_ok))

    print("[3/4] 落盘各级 PCD")
    sizes = {}
    if not args.no_offset_rung:
        RUNGS_LOCAL = RUNGS
    else:
        RUNGS_LOCAL = RUNGS[:-1]

    # R0 / R1 先做全局 1cm x K6 中位色
    pw0 = np.concatenate([d["pw"][d["m0"]] for d in data]) if data else np.zeros((0, 3))
    rgb0 = np.concatenate([d["rgbA"][d["m0"]] for d in data]) if data else np.zeros((0, 3))
    med0 = median_colors(pw0, rgb0) if len(pw0) else rgb0
    # 记录每帧在全局 R0 数组中的切片，供逐帧色差统计使用
    offs, s = [], 0
    for d in data:
        n = int(d["m0"].sum())
        offs.append((s, s + n))
        s += n
    w = PcdStream(os.path.join(args.out, "R0.bin"))
    w.add(pw0, med0)
    sizes["R0"] = w.finish(os.path.join(args.out, "R0.pcd"))
    w = PcdStream(os.path.join(args.out, "R1.bin"))
    w.add(pw0, rgb0)
    sizes["R1"] = w.finish(os.path.join(args.out, "R1.pcd"))

    for name, mk in [("R2", "m2"), ("R3", "m3"), ("R4", "m4")]:
        w = PcdStream(os.path.join(args.out, name + ".bin"))
        for d in data:
            m = d[mk]
            w.add(d["pw"][m], d["rgbA"][m])
        sizes[name] = w.finish(os.path.join(args.out, name + ".pcd"))

    w = PcdStream(os.path.join(args.out, "R5.bin"))
    for d in data:
        m = d["moff"]
        w.add(d["pw"][m], d["rgbO"][m])
    sizes["R5"] = w.finish(os.path.join(args.out, "R5.pcd"))

    if "R0b" in RUNGS_LOCAL:
        # R0b 与 R0 同样走 1cm x K6 中位，只差图像/位姿配对方式
        pw_b = np.concatenate([d["pw"][d["m0b"]] for d in data])
        rgb_b = np.concatenate([d["rgbB"][d["m0b"]] for d in data])
        medb = median_colors(pw_b, rgb_b) if len(pw_b) else rgb_b
        offs_b, sb = [], 0
        for d in data:
            n = int(d["m0b"].sum())
            offs_b.append((sb, sb + n))
            sb += n
        w = PcdStream(os.path.join(args.out, "R0b.bin"))
        w.add(pw_b, medb)
        sizes["R0b"] = w.finish(os.path.join(args.out, "R0b.pcd"))
    else:
        medb = None

    print("[4/4] 指标")
    # 色差 vs 官方参考（只在官方也着色的点上比较，保证各级同一批点）
    err_sum = {r: np.zeros(3) for r in RUNGS_LOCAL}
    err_cnt = {r: 0 for r in RUNGS_LOCAL}
    err_hist = {r: np.zeros(256, np.int64) for r in RUNGS_LOCAL}
    cover = {r: 0 for r in RUNGS_LOCAL}
    for i, d in enumerate(data):
        ref = d["ref"]
        if ref is None:
            continue
        has = d["moff"]
        a, b = offs[i]
        rgb_r0 = np.zeros((len(d["pw"]), 3), np.uint8)
        rgb_r0[d["m0"]] = med0[a:b]
        pairs = [("R0", d["m0"], rgb_r0),
                 ("R1", d["m0"], d["rgbA"]),
                 ("R2", d["m2"], d["rgbA"]),
                 ("R3", d["m3"], d["rgbA"]),
                 ("R4", d["m4"], d["rgbA"]),
                 ("R5", d["moff"], d["rgbO"])]
        if "R0b" in RUNGS_LOCAL:
            a2, b2 = offs_b[i]
            rgb_r0b = np.zeros((len(d["pw"]), 3), np.uint8)
            rgb_r0b[d["m0b"]] = medb[a2:b2]
            pairs.append(("R0b", d["m0b"], rgb_r0b))
        for name, m, rgb in pairs:
            if name not in err_sum:
                continue
            sel = m & has
            if not sel.any():
                continue
            dd = np.abs(rgb[sel].astype(np.int16) - ref[sel].astype(np.int16))
            err_sum[name] += dd.sum(0)
            err_cnt[name] += int(sel.sum())
            err_hist[name] += np.bincount(dd.ravel(), minlength=256)[:256]
    for i, d in enumerate(data):
        for name, mk in [("R0", "m0"), ("R1", "m0"), ("R2", "m2"),
                         ("R3", "m3"), ("R4", "m4"), ("R5", "moff"),
                         ("R0b", "m0b")]:
            if name in cover and mk in d:
                cover[name] += int(d[mk].sum())

    # 抽帧内部一致性（2/5cm 体素色 RMS）
    sub = data[::max(1, args.subsample)]
    rms = {}
    for name, mk, key in [("R1", "m0", "rgbA"), ("R2", "m2", "rgbA"),
                          ("R3", "m3", "rgbA"), ("R4", "m4", "rgbA"),
                          ("R5", "moff", "rgbO")]:
        if name not in RUNGS_LOCAL:
            continue
        xyz = np.concatenate([d["pw"][d[mk]] for d in sub])
        rgb = np.concatenate([d[key][d[mk]] for d in sub])
        rms[name] = (voxel_rms(xyz, rgb, 0.02), voxel_rms(xyz, rgb, 0.05))
    # R0 的中位色只在全局可用，这里按抽帧对应的全局切片取
    step = max(1, args.subsample)
    xyz = np.concatenate([data[i]["pw"][data[i]["m0"]] for i in range(0, len(data), step)])
    rgb = np.concatenate([med0[offs[i][0]:offs[i][1]] for i in range(0, len(data), step)])
    rms["R0"] = (voxel_rms(xyz, rgb, 0.02), voxel_rms(xyz, rgb, 0.05))
    if "R0b" in RUNGS_LOCAL:
        xyz = np.concatenate([data[i]["pw"][data[i]["m0b"]] for i in range(0, len(data), step)])
        rgb = np.concatenate([medb[offs_b[i][0]:offs_b[i][1]] for i in range(0, len(data), step)])
        rms["R0b"] = (voxel_rms(xyz, rgb, 0.02), voxel_rms(xyz, rgb, 0.05))

    lines = ["| 级 | 说明 | 点数 | 着色率 | vs官方色差MAE | 色差p95 | 色差>30占比 | 2cm RMS | 5cm RMS |",
             "|---|---|---|---|---|---|---|---|---|"]
    for name in RUNGS_LOCAL:
        mae = (err_sum[name] / max(1, err_cnt[name])).mean() if err_cnt[name] else float("nan")
        r2, r5 = rms.get(name, (float("nan"), float("nan")))
        h = err_hist[name]
        tot = max(1, int(h.sum()))
        cum = np.cumsum(h)
        p95 = float(np.searchsorted(cum, 0.95 * tot))
        big = 100.0 * float(h[31:].sum()) / tot
        lines.append("| %s | %s | %d | %.1f%% | %.2f | %.0f | %.2f%% | %.2f | %.2f |"
                     % (name, TITLES[name], sizes.get(name, 0),
                        100.0 * cover[name] / max(1, n_in), mae, p95, big, r2, r5))
    table = "\n".join(lines)
    print(table)
    with open(os.path.join(args.out, "ladder_metrics.md"), "w") as f:
        f.write(table + "\n")
    print("[done] %.1fs -> %s" % (time.time() - t0, args.out))


if __name__ == "__main__":
    main()
