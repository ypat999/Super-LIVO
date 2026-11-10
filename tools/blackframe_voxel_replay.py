#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""黑帧污染追踪 + median 消融：离线重放体素颜色融合（不改任何算法代码）。

原理：
  - 录制的 sl_frames bag 含 40 帧 colored_cloud（全量点，quality=0 的点 rgb=0）
    与 /lio/odom；sl_map 是最终地图（只 append quality>0 的点，颜色被 median 回填）。
  - median 池只收 quality>0 的点；bag 里没有 quality 字段，但地图几何 = 全部入池
    观测的世界系坐标，用 0.1mm 量化精确匹配即可逐点还原入池与否。
  - 之后完全按 VoxelColorMedian 的规则（1cm 体素、K=6 环形缓冲、逐通道中位数、
    插入顺序=帧序+帧内序）离线重放，做两组反事实：
      ① A=全部帧 / B=去掉黑帧 / C=仅黑帧
      ② A=时间中位(线上) / M=算术平均 / F=逐帧不融合

用法见 __main__。输出：逐体素观测清单 CSV.gz、各变体体素色 npz、summary.json。
"""
import argparse
import gzip
import json
import os
import re

import numpy as np
from scipy.ndimage import median_filter, convolve, distance_transform_edt
from scipy.spatial.transform import Rotation, Slerp

BLACK_LUM = 3.0      # 近黑亮度阈值（与 verify_black.py 一致）
JUMP_THR = 40.0      # 椒盐/跳变阈值（与 qc_validate_v1.salt_pepper 一致）
K_RING = 6           # VoxelColorMedian 环形缓冲长度
VOX = 0.01           # 着色体素边长（pcd.median_voxel_size 默认 1cm）
QMEM = 10000.0       # 地图成员判定量化：0.1 mm（插值复现误差 ~1e-7 m，可安全吸收）


# ------------------------------------------------------------------ 基础读取
def read_pcd_xyzrgb(path):
    with open(path, "rb") as f:
        head = b""
        while True:
            line = f.readline()
            head += line
            if line.lower().strip().startswith(b"data"):
                fmt = line.lower().strip().split()[-1]
                break
        assert fmt == b"binary", f"仅支持 binary PCD: {path}"
        txt = head.decode("ascii", "ignore")
        fields = re.search(r"FIELDS\s+(.+)", txt).group(1).split()
        sizes = [int(v) for v in re.search(r"SIZE\s+(.+)", txt).group(1).split()]
        counts = [int(v) for v in re.search(r"COUNT\s+(.+)", txt).group(1).split()]
        n = int(re.search(r"POINTS\s+(\d+)", txt).group(1))
        strides = [s * c for s, c in zip(sizes, counts)]
        step = sum(strides)
        buf = np.frombuffer(f.read(n * step), np.uint8).reshape(n, step)
    off = {}
    o = 0
    for name, st in zip(fields, strides):
        off[name] = (o, st)
        o += st

    def f4(name):
        s, w = off[name]
        return np.frombuffer(buf[:, s:s + w].copy().tobytes(), np.float32).reshape(n, w // 4)

    P = np.stack([f4("x")[:, 0], f4("y")[:, 0], f4("z")[:, 0]], 1)
    s, w = off["rgb"]
    rgb = np.frombuffer(buf[:, s:s + w].copy().tobytes(), np.uint32).ravel()
    return P.astype(np.float32), rgb


def load_bag(uri):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    frames, odom = [], []
    while r.has_next():
        topic, payload, _ = r.read_next()
        if not isinstance(payload, (bytes, bytearray)):
            continue
        if "colored_cloud" in topic:
            m = deserialize_message(payload, PointCloud2)
            n = m.width * m.height
            buf = np.frombuffer(m.data, np.uint8).reshape(n, m.point_step)
            fo = {f.name: f.offset for f in m.fields}
            P = np.frombuffer(buf[:, fo["x"]:fo["x"] + 12].tobytes(), np.float32).reshape(n, 3)
            pk = np.frombuffer(buf[:, fo["rgb"]:fo["rgb"] + 4].tobytes(), np.uint32).ravel()
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            frames.append((t, P.copy(), pk.copy()))
        elif "odom" in topic:
            m = deserialize_message(payload, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            odom.append((t, np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])))
    odom.sort(key=lambda s: s[0])
    return frames, odom


def make_interpolator(odom):
    stamps = np.array([o[0] for o in odom])
    pos = np.array([o[1] for o in odom])
    quat = np.array([o[2] for o in odom])
    slerp = Slerp(stamps, Rotation.from_quat(quat))

    def pose_at(t):
        if t <= stamps[0]:
            return Rotation.from_quat(quat[0]).as_matrix(), pos[0]
        if t >= stamps[-1]:
            return Rotation.from_quat(quat[-1]).as_matrix(), pos[-1]
        j = int(np.searchsorted(stamps, t))
        if stamps[j] == t:
            return Rotation.from_quat(quat[j]).as_matrix(), pos[j]
        R = slerp([t])[0].as_matrix()
        a = (t - stamps[j - 1]) / (stamps[j] - stamps[j - 1])
        return R, (1 - a) * pos[j - 1] + a * pos[j]

    return pose_at


# ------------------------------------------------------------------ 小工具
def lum_of(pk):
    return (0.114 * (pk & 0xFF) + 0.587 * ((pk >> 8) & 0xFF) + 0.299 * ((pk >> 16) & 0xFF))


def rgb_of(pk):
    return np.stack([((pk >> 16) & 0xFF).astype(np.uint8),
                     ((pk >> 8) & 0xFF).astype(np.uint8),
                     (pk & 0xFF).astype(np.uint8)], 1)


def lum_rgb3(rgb):
    return 0.299 * rgb[:, 0] + 0.587 * rgb[:, 1] + 0.114 * rgb[:, 2]


def pack_i64(q, bits):
    a = q.astype(np.int64) + (1 << (bits - 1))
    assert np.abs(q).max() < (1 << (bits - 1)), "量化坐标超界"
    return (a[:, 0] << (2 * bits)) | (a[:, 1] << bits) | a[:, 2]


def group_stat(inv, n_groups, sizes, rgb, k):
    """按组取插入序最后 <=k 个观测，逐通道中位数。行序=插入序。"""
    order = np.argsort(inv, kind="stable")
    inv_s, rgb_s = inv[order], rgb[order]
    starts = np.zeros(n_groups, np.int64)
    np.cumsum(sizes[:-1], out=starts[1:])
    gpos = np.arange(len(inv_s)) - starts[inv_s]
    maxk = min(k, int(sizes.max()))
    keep = gpos >= (sizes[inv_s] - maxk)
    slot = maxk - sizes[inv_s] + gpos
    out = np.zeros((n_groups, 3))
    for ch in range(3):
        pad = np.full((n_groups, maxk), np.nan)
        pad[inv_s[keep], slot[keep]] = rgb_s[keep, ch].astype(np.float64)
        out[:, ch] = np.nanmedian(pad, 1)
    return out


def group_mean(inv, n_groups, sizes, rgb):
    out = np.zeros((n_groups, 3))
    for ch in range(3):
        out[:, ch] = np.bincount(inv, weights=rgb[:, ch].astype(np.float64),
                                 minlength=n_groups) / sizes
    return out


def fuse(rows_rgb, rows_el, rows_frame, black_frames, mode):
    """返回 (uniq_vox, colors float64 (m,3), sizes)。mode: med6/medall/mean。
    rows_* 已按插入序拼接。"""
    if mode == "black_only":
        sel = np.isin(rows_frame, black_frames) & rows_el
    elif mode == "drop_black":
        sel = ~np.isin(rows_frame, black_frames) & rows_el
    else:
        sel = rows_el
    vox = np.floor(rows_xyz[sel] / VOX).astype(np.int32)
    if len(vox) == 0:
        return (np.zeros(0, np.int64), np.zeros((0, 3)), np.zeros(0, np.int64),
                vox, np.zeros(0, np.int64))
    key = pack_i64(vox, 16)
    uniq, inv = np.unique(key, return_inverse=True)
    sizes = np.bincount(inv, minlength=len(uniq))
    rgb = rows_rgb[sel]
    if mode in ("med6", "drop_black", "black_only"):
        col = group_stat(inv, len(uniq), sizes, rgb, K_RING)
    elif mode == "medall":
        col = group_stat(inv, len(uniq), sizes, rgb, int(sizes.max()))
    else:
        col = group_mean(inv, len(uniq), sizes, rgb)
    return uniq, col, sizes, vox, inv


# ------------------------------------------------------------------ 主流程
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", default="/home/ywj/slam_data/singleframe/sl_frames")
    ap.add_argument("--map", default="/home/ywj/slam_data/singleframe/sl_map/super_livo_visual_a_colored.pcd")
    ap.add_argument("--log", default="/tmp/sf_vis.log")
    ap.add_argument("--params", default="/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml")
    ap.add_argument("--out", default="/home/ywj/slam_data/singleframe/blackframe_replay")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    global rows_xyz
    rows_xyz = None

    frames, odom = load_bag(a.bag)
    print(f"bag: {len(frames)} 帧 colored_cloud, {len(odom)} 条 odom")
    pose_at = make_interpolator(odom)
    Pmap, rgbmap = read_pcd_xyzrgb(a.map)
    print(f"map: {len(Pmap)} 点")

    map_keys = pack_i64(np.round(Pmap * QMEM).astype(np.int32), 21)
    uniq_map, idx_map = np.unique(map_keys, return_index=True)

    log_counts = {}
    if os.path.exists(a.log):
        txt = open(a.log, "rb").read().decode("utf-8", "ignore")
        for m in re.finditer(r"t_cloud=([\d.]+).*? (\d+)/\d+ 点上色 帧#(\d+)", txt):
            log_counts[float(m.group(1))] = (int(m.group(2)), int(m.group(3)))

    # ---- 逐帧：世界系变换 + 入池判定 ---------------------------------------
    per_frame, xs = [], []
    for fi, (t, Pb, pk) in enumerate(frames):
        R, tv = pose_at(t)
        Pw = (Pb.astype(np.float64) @ R.T + tv).astype(np.float32)
        keys = pack_i64(np.round(Pw * QMEM).astype(np.int32), 21)
        hit = np.searchsorted(uniq_map, keys)
        hit_c = np.clip(hit, 0, len(uniq_map) - 1)
        el = uniq_map[hit_c] == keys
        rgb = rgb_of(pk)
        nb = lum_of(pk) < BLACK_LUM
        n_el = int(el.sum())
        lc = log_counts.get(round(t, 4))
        near_el = int((nb & el).sum())
        per_frame.append(dict(frame=fi, t_cloud=t, n=len(Pb), eligible=n_el,
                              near_black_all=int(nb.sum()), near_black_elig=near_el,
                              log_colored=lc[0] if lc else None,
                              log_frameno=lc[1] if lc else None))
        xs.append((Pw, rgb, el, nb, np.full(len(Pb), fi, np.int32)))
        print(f"帧#{fi:2d} t={t:.4f} 点{len(Pb):6d} 入池{n_el:6d} 池内近黑{near_el:6d} "
              f"日志上色={lc[0] if lc else '-'}")

    diffs = [(r["eligible"], r["log_colored"]) for r in per_frame if r["log_colored"] is not None]
    if diffs:
        d = np.abs(np.array(diffs)[:, 0] - np.array(diffs)[:, 1])
        print(f"[校验] 入池数 vs 日志上色数: max 差 {d.max()}, mean {d.mean():.2f} (n={len(d)})")

    black_frames = [r["frame"] for r in per_frame
                    if r["eligible"] > 0 and r["near_black_elig"] / r["eligible"] >= 0.9]
    empty_frames = [r["frame"] for r in per_frame if r["eligible"] == 0]
    print(f"黑帧(池内近黑>=90%): {black_frames}; 0 上色帧: {empty_frames}")

    # 0 上色帧：它们是否仍进入了最终地图几何？（同帧同坐标=同一物理点）
    zero_in_map = {}
    for fi in empty_frames:
        t, Pb, pk = frames[fi]
        R, tv = pose_at(t)
        Pw = (Pb.astype(np.float64) @ R.T + tv).astype(np.float32)
        keys = pack_i64(np.round(Pw * QMEM).astype(np.int32), 21)
        hit = np.searchsorted(uniq_map, keys)
        el = uniq_map[np.clip(hit, 0, len(uniq_map) - 1)] == keys
        zero_in_map[fi] = dict(n=int(len(Pw)), in_map=int(el.sum()))
        print(f"0上色帧#{fi}: {int(el.sum())}/{len(Pw)} 点坐标出现在最终地图里")

    rows_xyz = np.concatenate([x[0] for x in xs])
    rows_rgb = np.concatenate([x[1] for x in xs])
    rows_el = np.concatenate([x[2] for x in xs])
    rows_nb = np.concatenate([x[3] for x in xs])
    rows_fr = np.concatenate([x[4] for x in xs])

    # ---- 观测清单落盘 -------------------------------------------------------
    vox_all = np.floor(rows_xyz / VOX).astype(np.int32)
    vid = pack_i64(vox_all, 16)
    arr = np.column_stack([vid, rows_fr, np.arange(len(vid)),
                           rows_xyz, rows_rgb, rows_nb.astype(np.int8),
                           rows_el.astype(np.int8)])
    with gzip.open(os.path.join(a.out, "voxel_observations.csv.gz"), "wt") as fo:
        fo.write("voxel_id,frame_id,point_id,x,y,z,r,g,b,is_near_black,pool_eligible\n")
        np.savetxt(fo, arr, delimiter=",", fmt=["%d", "%d", "%d", "%.4f", "%.4f", "%.4f",
                                                "%d", "%d", "%d", "%d", "%d"])
    print(f"观测清单: {len(arr)} 行 -> voxel_observations.csv.gz")

    # ---- 融合变体 ------------------------------------------------------------
    variants = {}
    for name, mode in [("A_med6", "all"), ("B_drop_black", "drop_black"),
                       ("C_black_only", "black_only"), ("M_mean", "meanall")]:
        m = mode if mode != "meanall" else "mean"
        uniq, col, sizes, vox, inv = fuse(rows_rgb, rows_el, rows_fr, black_frames, m if mode != "all" else "med6")
        variants[name] = (uniq, col, sizes, vox, inv)
    # medall 参考版（判断 K=6 环形 vs 全量中位差异）
    uniq, col, sizes, vox, inv = fuse(rows_rgb, rows_el, rows_fr, black_frames, "medall")
    variants["A_medall"] = (uniq, col, sizes, vox, inv)

    # ---- ① A vs B vs C ------------------------------------------------------
    ua, ca, sa, _, _ = variants["A_med6"]
    ub, cb, sb, _, _ = variants["B_drop_black"]
    uc, cc, sc, _, _ = variants["C_black_only"]
    ia = np.searchsorted(ub, ua); ok = (ia < len(ub)) & (ub[np.clip(ia, 0, len(ub)-1)] == ua)
    la_a, la_b = lum_rgb3(ca), lum_rgb3(cb)
    d_lum = np.zeros(len(ua)); d_lum[ok] = np.abs(la_a[ok] - la_b[ia[ok]])
    black_a = la_a < BLACK_LUM
    black_b = np.zeros(len(ua), bool); black_b[ok] = la_b[ia[ok]] < BLACK_LUM
    flip_to_black = int((black_a[ok] & ~black_b[ia[ok]]).sum())
    flip_from_black = int(((~black_a)[ok] & black_b[ia[ok]]).sum())

    # 每个 A 体素里黑帧观测占比
    if len(black_frames):
        bf_mask = np.isin(rows_fr, black_frames) & rows_el
        key_bf = pack_i64(np.floor(rows_xyz[bf_mask] / VOX).astype(np.int32), 16)
        ubf, cbf = np.unique(key_bf, return_counts=True)
        j = np.searchsorted(ubf, ua); okj = (j < len(ubf)) & (ubf[np.clip(j, 0, len(ubf)-1)] == ua)
        n_bf = np.zeros(len(ua), np.int64); n_bf[okj] = cbf[j[okj]]
    else:
        n_bf = np.zeros(len(ua), np.int64)
    frac_bf = n_bf / np.maximum(sa, 1)
    has_bf = n_bf > 0
    hist = [int(((frac_bf > lo) & (frac_bf <= hi) & has_bf).sum())
            for lo, hi in [(0, .2), (.2, .4), (.4, .5), (.5, .7), (.7, 1.0)]]

    # ---- 保真校验：窗口重放 A_med6 vs 线上地图保存色 -------------------------
    sel_el = rows_el
    vox_el = pack_i64(np.floor(rows_xyz[sel_el] / VOX).astype(np.int32), 16)
    jj = np.searchsorted(ua, vox_el)
    okk = (jj < len(ua)) & (ua[np.clip(jj, 0, len(ua)-1)] == vox_el)
    keys_pt = pack_i64(np.round(rows_xyz[sel_el] * QMEM).astype(np.int32), 21)
    jm = np.searchsorted(uniq_map, keys_pt)
    saved_rgb = rgbmap[idx_map[np.clip(jm, 0, len(uniq_map)-1)]]
    d_saved = np.abs(lum_rgb3(ca[jj[okk]]) - lum_of(saved_rgb[okk]))
    agree = float((d_saved <= 8.0).mean())

    # ---- ② 各变体体素域指标 --------------------------------------------------
    def voxel_metrics(tag, uniq, col, sizes, vox, inv):
        L = lum_rgb3(col)
        nbv = float((L < BLACK_LUM).mean()) if len(L) else float("nan")
        # 体素内 RGB 标准差（intra_std 已只含观测>=2 的体素）
        if mode_std[tag] is not None and len(mode_std[tag]):
            iv_std = float(np.mean(mode_std[tag]))
        else:
            iv_std = float("nan")
        j, t = neighbor_jump(uniq, L, vox, inv)
        return dict(variant=tag, n_voxels=int(len(uniq)),
                    near_black_voxel_frac=round(nbv, 4),
                    mean_intra_voxel_lum_std=round(iv_std, 2),
                    neighbor_jump_frac=round(j / t, 4) if t else float("nan"))

    def neighbor_jump(uniq, L, vox, inv):
        """融合色体素的 +x/+y/+z 邻接对亮度跳变（|Δlum|>40），返回 (jumps, tot)。"""
        if len(uniq) == 0:
            return 0, 0
        vrep = np.zeros((len(uniq), 3), np.int32)
        vrep[inv] = vox  # 每个唯一体素取任一代表行
        jumps = 0
        tot = 0
        for d in [(1, 0, 0), (0, 1, 0), (0, 0, 1)]:
            nb = vrep + np.array(d)
            if np.abs(nb).max() >= (1 << 15):
                continue
            nk = pack_i64(nb, 16)
            jn = np.searchsorted(uniq, nk)
            jn_c = np.clip(jn, 0, len(uniq) - 1)
            hit = uniq[jn_c] == nk
            dd = np.abs(L[jn_c[hit]] - L[np.arange(len(uniq))][hit])
            jumps += int((dd > JUMP_THR).sum())
            tot += int(hit.sum())
        return jumps, tot

    # 体素内亮度标准差（按变体观测集）
    mode_std = {"A_med6": None, "B_drop_black": None, "C_black_only": None,
                "M_mean": None, "per_frame": None}

    def intra_std(sel):
        if not np.any(sel):
            return np.zeros(0)
        vox = pack_i64(np.floor(rows_xyz[sel] / VOX).astype(np.int32), 16)
        uq, inv = np.unique(vox, return_inverse=True)
        n = np.bincount(inv)
        L = lum_rgb3(rows_rgb[sel])
        s1 = np.bincount(inv, weights=L, minlength=len(uq))
        s2 = np.bincount(inv, weights=L * L, minlength=len(uq))
        mean = s1 / n
        var = np.maximum(s2 / n - mean ** 2, 0)
        m = n >= 2
        return np.sqrt(var[m])

    selA = rows_el
    selB = rows_el & ~np.isin(rows_fr, black_frames)
    selC = rows_el & np.isin(rows_fr, black_frames)
    mode_std["A_med6"] = intra_std(selA)
    mode_std["B_drop_black"] = intra_std(selB)
    mode_std["C_black_only"] = intra_std(selC)
    mode_std["M_mean"] = mode_std["A_med6"]
    mode_std["per_frame"] = intra_std(rows_el)  # 逐帧=同观测集（融合前）

    table = []
    for tag in ["A_med6", "B_drop_black", "C_black_only", "M_mean"]:
        u, c, s, v, i = variants[tag]
        table.append(voxel_metrics(tag, u, c, s, v, i))
    # 逐帧变体：体素=帧内 1cm 格（无跨帧融合），色=该帧该格观测均值
    vox_allf = np.floor(rows_xyz[rows_el] / VOX).astype(np.int32)
    kf = pack_i64(vox_allf, 16) * 100 + rows_fr[rows_el]
    uqf, invf = np.unique(kf, return_inverse=True)
    nf = np.bincount(invf)
    Lf = lum_rgb3(rows_rgb[rows_el])
    mf = np.bincount(invf, weights=Lf, minlength=len(uqf)) / nf
    # 逐帧邻跳变：每帧内部 1cm 格 +x/+y/+z 邻接（帧内几何，无跨帧混入）
    nj_num, nj_den = 0, 0
    fr_el = rows_fr[rows_el]
    for fi in range(len(frames)):
        mfi = fr_el == fi
        if not np.any(mfi):
            continue
        vox_f = vox_allf[mfi]
        uq1, inv1 = np.unique(pack_i64(vox_f, 16), return_inverse=True)
        n1 = np.bincount(inv1)
        L1 = np.bincount(inv1, weights=Lf[mfi], minlength=len(uq1)) / n1
        j1, t1 = neighbor_jump(uq1, L1, vox_f, inv1)
        nj_num += j1
        nj_den += t1
    table.append(dict(variant="per_frame", n_voxels=int(len(uqf)),
                      near_black_voxel_frac=round(float((mf < BLACK_LUM).mean()), 4),
                      mean_intra_voxel_lum_std=round(float(np.mean(mode_std["per_frame"])), 2),
                      neighbor_jump_frac=round(nj_num / nj_den, 4) if nj_den else float("nan")))

    # ---- 栅格椒盐（z-buffer，几何相同只换色） --------------------------------
    yaml = open(a.params).read()
    fx = float(re.search(r"cam_fx:\s*([\d.]+)", yaml).group(1))
    fy = float(re.search(r"cam_fy:\s*([\d.]+)", yaml).group(1))
    cx = float(re.search(r"cam_cx:\s*([\d.]+)", yaml).group(1))
    cy = float(re.search(r"cam_cy:\s*([\d.]+)", yaml).group(1))
    W = int(re.search(r"cam_width:\s*(\d+)", yaml).group(1))
    H = int(re.search(r"cam_height:\s*(\d+)", yaml).group(1))
    Rcl = np.array([float(v) for v in re.search(
        r"Rcl:\s*\[([^\]]+)\]", yaml, re.S).group(1).replace("\n", " ").split(",")])
    Pcl = np.array([float(v) for v in re.search(r"Pcl:\s*\[([^\]]+)\]", yaml).group(1).split(",")])
    Rcl = Rcl.reshape(3, 3)  # 注意：yaml 的 Rcl/Pcl = T_cam_lidar（p_cam=Rcl·p_lidar+Pcl）

    def raster_sp(color_fn_per_point):
        """color_fn_per_point(Pw)->(n,3)：None=逐帧色（只投当前帧点）；
        否则=融合色（把全部窗口入池点从每个相机位姿投影）。
        公平掩码：椒盐率只在"当前帧点云 5x5 全覆盖"的像素上统计，
        避免逐帧变体因覆盖稀疏虚高。返回平均椒盐率。"""
        Pw_el = rows_xyz[rows_el]
        rgb_fused = None if color_fn_per_point is None else color_fn_per_point(Pw_el)
        sps, ndens = [], []
        for fi, (t, Pb, pk) in enumerate(frames):
            R, tv = pose_at(t)
            Pw = (Pb.astype(np.float64) @ R.T + tv).astype(np.float32)
            keys = pack_i64(np.round(Pw * QMEM).astype(np.int32), 21)
            hit = np.searchsorted(uniq_map, keys)
            el = uniq_map[np.clip(hit, 0, len(uniq_map) - 1)] == keys
            if color_fn_per_point is None:
                P = Pw[el].astype(np.float64)
                rgb = rgb_of(pk[el])
            else:
                P, rgb = Pw_el.astype(np.float64), rgb_fused
            # 当前帧入池点的覆盖掩码（行向量式 p_l=(p_w-t)@R，再 p_cam=Rcl·p_l+Pcl）
            p_own = Pw[el].astype(np.float64)
            p_own = (p_own - tv) @ R @ Rcl.T + Pcl
            zo = p_own[:, 2]
            mo = zo > 0.3
            uo = fx * p_own[mo, 0] / zo[mo] + cx
            vo = fy * p_own[mo, 1] / zo[mo] + cy
            mm2 = (uo >= 2) & (uo < W - 2) & (vo >= 2) & (vo < H - 2)
            cov = np.zeros((H, W), bool)
            cov[np.floor(vo[mm2]).astype(np.int64), np.floor(uo[mm2]).astype(np.int64)] = True
            cc = convolve(cov.astype(np.float32), np.ones((5, 5)), mode="constant")
            dense = cc >= 1  # 当前帧覆盖 5x5 膨胀：所有变体在同一像素集上评估
            if dense.sum() < 200:
                continue
            p_cam = (P - tv) @ R
            p_cam = p_cam @ Rcl.T + Pcl
            z = p_cam[:, 2]
            m = (z > 0.3)
            u = fx * p_cam[m, 0] / z[m] + cx
            v = fy * p_cam[m, 1] / z[m] + cy
            rgb_m = rgb[m]
            mm = (u >= 1) & (u < W - 1) & (v >= 1) & (v < H - 1)
            u, v, rgb_m = u[mm], v[mm], rgb_m[mm]
            if len(u) < 1000:
                continue
            iu = np.floor(u).astype(np.int64); iv = np.floor(v).astype(np.int64)
            pix = iv * W + iu
            dn = z[m][mm]
            zb = np.full(W * H, np.inf)
            np.minimum.at(zb, pix, dn)
            order = np.argsort(-dn)
            flat = np.zeros(W * H)
            flat[pix[order]] = lum_rgb3(rgb_m)[order]
            L = flat.reshape(H, W)
            filled = np.isfinite(zb).reshape(H, W)
            _, near = distance_transform_edt(~filled, return_indices=True)
            L = L[near[0], near[1]]  # EDT 补洞：空格取最近有色像素（与 qc_validate_v1 一致）
            med = median_filter(L, 5, mode="nearest")
            sp = float(np.mean((np.abs(L - med) > JUMP_THR)[dense]))
            sps.append(sp)
            ndens.append(int(dense.sum()))
        return float(np.mean(sps)) if sps else float("nan")

    def color_from_variant(tag):
        u, c, s, v, _ = variants[tag]
        def fn(Pw):
            out = np.zeros((len(Pw), 3))
            if len(Pw) == 0:
                return out
            vox = pack_i64(np.floor(Pw / VOX).astype(np.int32), 16)
            j = np.searchsorted(u, vox)
            j_c = np.clip(j, 0, len(u) - 1)
            hit = u[j_c] == vox
            out[hit] = c[j_c[hit]]
            return out
        return fn

    sp_map = {}
    for tag in ["A_med6", "B_drop_black", "M_mean"]:
        sp_map[tag] = raster_sp(color_from_variant(tag))
    sp_map["per_frame"] = raster_sp(None)  # 逐帧色
    for row in table:
        row["raster_salt_pepper"] = round(sp_map.get(row["variant"], float("nan")), 4)

    # ---- 汇总输出 ------------------------------------------------------------
    summary = dict(
        per_frame=per_frame,
        black_frames=black_frames, empty_frames=empty_frames,
        zero_color_points_in_map=zero_in_map,
        counterfactual=dict(
            voxels_A=int(len(ua)), voxels_B=int(len(ub)), voxels_C=int(len(uc)),
            changed_A_vs_B=int((d_lum[ok] > 0).sum()),
            d_lum_mean=round(float(d_lum[ok].mean()), 2),
            d_lum_p90=round(float(np.percentile(d_lum[ok], 90)), 2),
            d_lum_max=round(float(d_lum[ok].max()), 1),
            near_black_voxels_A=int(black_a.sum()),
            near_black_voxels_B=int(black_b.sum()),
            near_black_voxels_C=int((lum_rgb3(cc) < BLACK_LUM).sum()),
            A_black_B_normal=flip_to_black, A_normal_B_black=flip_from_black,
            voxels_with_black_obs=int(has_bf.sum()),
            black_frac_hist_0_20_40_50_70_100=hist,
            voxels_black_frac_gt50=int((frac_bf > 0.5).sum()),
            replay_vs_saved_lum_le8_agree=round(agree, 4),
            replay_vs_saved_median_d=round(float(np.median(d_saved)), 2)),
        variants=table)
    with open(os.path.join(a.out, "summary.json"), "w") as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)

    print("\n=== ① 反事实 A vs B（K=6 中位） ===")
    for k, v in summary["counterfactual"].items():
        print(f"  {k}: {v}")
    print("\n=== ② 变体指标 ===")
    print(f"  {'variant':<14}{'nvox':>9}{'近黑体素':>9}{'体内std':>9}{'邻跳变':>8}{'栅格椒盐':>9}")
    for r in table:
        print(f"  {r['variant']:<14}{r['n_voxels']:>9}{r['near_black_voxel_frac']:>9}"
              f"{r['mean_intra_voxel_lum_std']:>9}{r['neighbor_jump_frac']:>8}"
              f"{r['raster_salt_pepper']:>9}")


if __name__ == "__main__":
    main()
