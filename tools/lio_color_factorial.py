#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LIO×着色 排列组合实验（纯离线，不改任何生产代码）。

用户假设：椒盐的根源在 LIO 精度 —— 无色原始点云层面就发散，上色只是让它
更可见。检验方法：把"位姿/几何质量"（L 因子）与"着色规则"（C 因子）解耦，
做全组合矩阵，看最终指标随哪个因子变化。

L 因子（9 级）：
  sl_clean        —— Super-LIVO 原始 odom（基线，应与 §13.2 吻合）
  drift_{3,10,25} —— 随机游走位姿扰动：共模漂移（相邻帧相对误差小）
  jit_{3,10,25}   —— 逐帧独立抖动：差模误差（相邻帧相对误差=√2 倍）
  e1_real         —— 真实 LIO #2（E1 跑，已知 ATE≈21cm vs sl）
  fl2_official    —— 真实 LIO #3（官方 FAST-LIVO2 世界系彩色逐帧云）
扰动实现：对 40 帧各自的 (R,t) 加噪声后重建观测世界坐标；投影/栅格用同一
扰动位姿（模拟该位姿下的真实运行），观测集与资格固定 → 唯一变量=位姿。

C 因子（3 模型）：
  F 逐观测自身色（纯追加）/ A K=6 体素中位数（现行）/ G 一致性门控(5mm,40)

每格指标：融合地图视图栅格椒盐率（§13 同口径）。
每 L 级无色层指标：多帧同体素碰撞率、跨帧质心 spread（中位/p90）、
局部平面残差 plane_rms（=“点云发散/厚度”的直接量）、实测位姿偏差。
"""
import json
import os
import re
import sys
import time

import numpy as np
from scipy.ndimage import median_filter, convolve, distance_transform_edt
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from voxel_attribution import load_obs, voxel_table, pack16, OBS, VOX
from blackframe_voxel_replay import (load_bag, make_interpolator, rgb_of,
                                     group_stat, lum_rgb3, read_pcd_xyzrgb, QMEM)
from voxel_fusion_G import fit_planes, run_G, JUMP_THR, K_RING

OUT = "/home/ywj/slam_data/singleframe/blackframe_replay/lio_color_factorial"
PARAMS = "/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml"
BAGS = {"sl": "/home/ywj/slam_data/singleframe/sl_frames",
        "e1": "/home/ywj/slam_data/e1_append/e1_frames",
        "fl2": "/home/ywj/slam_data/singleframe/fl2_frames"}
SEED = 20261109
MAXF = 40          # 每级帧数上限
D_THR, C_THR = 0.005, 40


def cam_params():
    y = open(PARAMS).read()
    return (float(re.search(r"cam_fx:\s*([\d.]+)", y).group(1)),
            float(re.search(r"cam_fy:\s*([\d.]+)", y).group(1)),
            float(re.search(r"cam_cx:\s*([\d.]+)", y).group(1)),
            float(re.search(r"cam_cy:\s*([\d.]+)", y).group(1)),
            int(re.search(r"cam_width:\s*(\d+)", y).group(1)),
            int(re.search(r"cam_height:\s*(\d+)", y).group(1)),
            np.array([float(v) for v in re.search(r"Rcl:\s*\[([^\]]+)\]", y, re.S)
                      .group(1).replace("\n", " ").split(",")]).reshape(3, 3),
            np.array([float(v) for v in re.search(r"Pcl:\s*\[([^\]]+)\]", y)
                      .group(1).split(",")]))


# ---------------------------------------------------------------- 载入各源
def load_fl2():
    """官方 FAST-LIVO2 bag：/cloud_registered（世界系，rgb=float32 位模式）
    + /aft_mapped_to_init（odom）。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAGS["fl2"], storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    frames, odom = [], []
    while r.has_next():
        topic, payload, _ = r.read_next()
        if not isinstance(payload, (bytes, bytearray)):
            continue
        if "cloud_registered" in topic:
            m = deserialize_message(payload, PointCloud2)
            n = m.width * m.height
            if n == 0:
                continue
            buf = np.frombuffer(m.data, np.uint8).reshape(n, m.point_step)
            fo = {f.name: f.offset for f in m.fields}
            P = np.frombuffer(buf[:, fo["x"]:fo["x"] + 12].tobytes(),
                              np.float32).reshape(n, 3)
            pk = np.frombuffer(buf[:, fo["rgb"]:fo["rgb"] + 4].tobytes(),
                               np.uint32).ravel()
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            frames.append((t, P.copy(), pk.copy()))
        elif "mapped" in topic:
            m = deserialize_message(payload, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            odom.append((t, np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])))
    odom.sort(key=lambda s: s[0])
    return frames, odom


def load_source(src):
    """返回 frames: list of (t, P_body_or_world float32 (n,3), rgb uint32),
    odom list。e1/fl2 已按帧筛选/抽样。"""
    if src == "fl2":
        frames, odom = load_fl2()
        return frames[::4][:MAXF], odom
    frames, odom = load_bag(BAGS[src])
    if src == "sl":
        return frames[:MAXF], odom
    if src == "e1":
        # 均匀覆盖全程 ~22s（与 sl 同窗同帧数）。早期版本用 rgb!=0>=5000
        # 过滤导致 40 帧挤在前 6.6s，密度 5 倍混杂不可比——已修正。
        idx = np.linspace(0, len(frames) - 1, MAXF).astype(int)
        return [frames[i] for i in idx], odom
    raise ValueError(src)


def pooled_sl():
    """sl 级观测集：与 §13 完全一致（CSV pool_eligible），返回按帧分组。"""
    vid, fr, P, RGB, EL = load_obs(OBS)
    return fr[EL], P[EL], RGB[EL]


# ---------------------------------------------------------------- 位姿扰动
def frame_poses(frames, odom, mode="clean", pos=0.0, rot_deg=0.0):
    """每帧干净 (R,t)（插值），再按模式加噪。返回 lists + 偏差统计。"""
    pose_at = make_interpolator(odom)
    clean = [pose_at(t) for (t, _, _) in frames]
    nf = len(frames)
    rng = np.random.default_rng(SEED)
    if mode == "drift":
        dp = rng.normal(0, pos / np.sqrt(max(nf - 1, 1)), (nf, 3))
        dp[0] = 0.0
        dp = np.cumsum(dp, 0)
        dr = rng.normal(0, np.deg2rad(rot_deg) / np.sqrt(max(nf - 1, 1)), (nf, 3))
        dr[0] = 0.0
        dr = np.cumsum(dr, 0)
    elif mode == "jit":
        dp = rng.normal(0, pos, (nf, 3))
        dr = rng.normal(0, np.deg2rad(rot_deg), (nf, 3))
    else:
        dp = np.zeros((nf, 3)); dr = np.zeros((nf, 3))
    out = []
    for i, (R, tv) in enumerate(clean):
        Rp = Rotation.from_rotvec(dr[i]).as_matrix() @ R
        out.append((Rp, tv + dp[i]))
    dev = np.linalg.norm(dp, axis=1)
    rel = np.linalg.norm(np.diff(dp, axis=0), axis=1) if nf > 1 else np.zeros(1)
    relR = np.rad2deg(np.linalg.norm(np.diff(dr, axis=0), axis=1)) if nf > 1 else np.zeros(1)
    # 全体帧对平均相对位姿误差（着色真正面对的失配量：漂移与抖动都归一到此）
    pd = np.linalg.norm(dp[:, None, :] - dp[None, :, :], axis=2)
    pr = np.linalg.norm(dr[:, None, :] - dr[None, :, :], axis=2)
    iu = np.triu_indices(nf, 1)
    stats = dict(pose_dev_mean_cm=float(dev.mean() * 100),
                 pose_dev_max_cm=float(dev.max() * 100),
                 rel_trans_mean_cm=float(rel.mean() * 100),
                 rel_rot_mean_deg=float(relR.mean()),
                 pairwise_trans_cm=float(pd[iu].mean() * 100) if nf > 1 else 0.0,
                 pairwise_rot_deg=float(np.rad2deg(pr[iu].mean())) if nf > 1 else 0.0)
    return out, stats, clean


# ---------------------------------------------------------------- 栅格评价
def raster_eval(Pw, RGB, fr, starts, poses, cam, lumA, lumG, n, uniq):
    fx, fy, cx, cy, W, H, Rcl, Pcl = cam
    allvox = np.floor(Pw / VOX).astype(np.int32)
    allg = np.searchsorted(uniq, pack16(allvox))
    L_el = lum_rgb3(RGB)
    imb = n[allg] >= 2
    tot = {"F": 0, "A": 0, "G": 0}
    mlt = {"F": 0, "A": 0, "G": 0}
    covsum = 0
    for fi, (R, tv) in enumerate(poses):
        b0, b1 = starts[fi], starts[fi + 1]
        if b1 - b0 < 200:
            continue
        Po = Pw[b0:b1]
        p_own = (Po - tv) @ R @ Rcl.T + Pcl
        zo = p_own[:, 2]; mo = zo > 0.3
        uo = fx * p_own[mo, 0] / zo[mo] + cx
        vo = fy * p_own[mo, 1] / zo[mo] + cy
        okc = (uo >= 2) & (uo < W - 2) & (vo >= 2) & (vo < H - 2)
        cov = np.zeros((H, W), bool)
        cov[np.floor(vo[okc]).astype(np.int64), np.floor(uo[okc]).astype(np.int64)] = True
        cc = convolve(cov.astype(np.float32), np.ones((5, 5)), mode="constant") >= 1
        p_cam = (Pw - tv) @ R @ Rcl.T + Pcl
        z = p_cam[:, 2]; m = z > 0.3
        u = fx * p_cam[m, 0] / z[m] + cx
        v = fy * p_cam[m, 1] / z[m] + cy
        mm = (u >= 1) & (u < W - 1) & (v >= 1) & (v < H - 1)
        u, v, zz = u[mm], v[mm], z[m][mm]
        gidx = allg[m][mm]
        Lown = L_el[m][mm]
        im = np.floor(v).astype(np.int64) * W + np.floor(u).astype(np.int64)
        zb = np.full(W * H, np.inf)
        np.minimum.at(zb, im, zz)
        filled = np.isfinite(zb)
        win = np.full(W * H, -1, np.int64)
        ordz = np.argsort(-zz)
        win[im[ordz]] = np.arange(len(im))[ordz]
        wi = win[filled]
        gwin = gidx[wi]
        Mfull = np.zeros(W * H, bool)
        Mfull[filled] = imb[wi]
        _, near = distance_transform_edt(~filled.reshape(H, W), return_indices=True)
        Mf = Mfull.reshape(H, W)[near[0], near[1]]
        covsum += int(cc.sum())
        src = {"F": Lown[wi], "A": lumA[gwin], "G": lumG[gwin]}
        for k, Lw in src.items():
            Lpix = np.zeros(W * H)
            Lpix[filled] = Lw
            Lf = Lpix.reshape(H, W)[near[0], near[1]]
            jump = (np.abs(Lf - median_filter(Lf, 5, mode="nearest")) > JUMP_THR) & cc
            tot[k] += int(jump.sum())
            mlt[k] += int((jump & Mf).sum())
    return {k: dict(pixels=tot[k], rate=tot[k] / max(covsum, 1) * 100,
                    on_multi=mlt[k]) for k in tot}, covsum


# ---------------------------------------------------------------- 无色层指标
def geo_metrics(Pw, fr, n, uniq, inv, starts, order, maxmf=40000):
    """多帧碰撞率 / 跨帧质心 spread / 局部平面残差。"""
    # 逐 (voxel,frame) 计数 → 该体素被几个不同帧观测
    key2 = inv.astype(np.int64) * 100000 + fr.astype(np.int64)
    u2 = np.unique(key2)
    nvf = np.bincount(u2 // 100000, minlength=len(uniq))
    mf = np.where((n >= 2) & (nvf >= 2))[0]
    in_mf = np.zeros(len(uniq), bool); in_mf[mf] = True
    collide_frac = float(in_mf[inv].mean())
    rng = np.random.default_rng(1)
    mfs = rng.choice(mf, size=min(len(mf), maxmf), replace=False) if len(mf) else mf
    spreads = []
    for g in mfs:
        r0, nn = starts[g], n[g]
        rid = order[r0:r0 + nn]
        P, F = Pw[rid], fr[rid]
        f_ids = np.unique(F)
        if len(f_ids) < 2:
            continue
        cents = np.stack([P[F == f].mean(0) for f in f_ids])
        dd = np.linalg.norm(cents[:, None] - cents[None, :], axis=2)
        spreads.append(dd.max())
    spreads = np.array(spreads) if spreads else np.zeros(1)
    cent = np.stack([np.bincount(inv, weights=Pw[:, c], minlength=len(uniq)) / n
                     for c in range(3)], 1)
    c0, nrm, ok = fit_planes(uniq, cent, n, mfs)
    d_rms = []
    for gi, g in enumerate(mfs):
        if not ok[gi]:
            continue
        r0, nn = starts[g], n[g]
        rid = order[r0:r0 + nn]
        d = (Pw[rid] - c0[gi]) @ nrm[gi]
        d_rms.append(d ** 2)
    plane_rms = float(np.sqrt(np.mean(np.concatenate(d_rms))) * 100) if d_rms else float("nan")
    return dict(multi_frame_voxels=int(len(mf)), collide_frac=collide_frac,
                spread_med_cm=float(np.median(spreads) * 100),
                spread_p90_cm=float(np.percentile(spreads, 90) * 100),
                plane_rms_cm=plane_rms, mfs=int(len(mfs)))


# ---------------------------------------------------------------- 单级流水线
def build_level(name, src, mode, pos, rot_deg, sl_pool, cache):
    frames, odom = cache[src]
    poses, pstats, clean = frame_poses(frames, odom, mode, pos, rot_deg)
    if src == "sl":
        # 观测=CSV pool（世界坐标=干净位姿产物），恢复体系后按扰动位姿重建
        fr0, P0, RGB0 = sl_pool
        nf = len(frames)
        # CSV frame_id -> 帧索引（sl 用前 MAXF 帧；CSV 帧序与 bag 一致）
        Pb = np.empty_like(P0)
        for fi in range(nf):
            m = fr0 == fi
            if not m.any():
                continue
            R, tv = clean[fi]
            Pb[m] = (P0[m] - tv) @ R          # 逆 Pw = Pb@R.T + tv
        Pw = np.empty_like(P0)
        for fi in range(nf):
            m = fr0 == fi
            if not m.any():
                continue
            R, tv = poses[fi]
            Pw[m] = Pb[m] @ R.T + tv
        fr = fr0
    else:
        parts_f, parts_P, parts_C = [], [], []
        off = 0
        for fi, (t, P, pk) in enumerate(frames):
            R, tv = clean[fi] if src == "e1" else (np.eye(3), np.zeros(3))
            Pw_f = P.astype(np.float64) @ R.T + tv
            rgb = pk.astype(np.uint32)
            if src == "e1":
                keep = rgb != 0
                Pw_f, rgb = Pw_f[keep], rgb[keep]
            parts_f.append(np.full(len(Pw_f), fi, np.int32))
            parts_P.append(Pw_f)
            parts_C.append(rgb)
        fr = np.concatenate(parts_f)
        Pw = np.concatenate(parts_P)
        rgb_pk = np.concatenate(parts_C)
        RGB0 = rgb_of(rgb_pk)
    starts = np.zeros(len(frames) + 1, np.int64)
    np.add.at(starts[1:], fr, 1)
    np.cumsum(starts[1:], out=starts[1:])
    RGB = RGB0.astype(np.float64)

    vox = np.floor(Pw / VOX).astype(np.int32)
    key = pack16(vox)
    uniq, inv = np.unique(key, return_inverse=True)
    n = np.bincount(inv, minlength=len(uniq))
    order = np.argsort(inv, kind="stable")
    gstarts = np.zeros(len(uniq), np.int64)
    np.cumsum(n[:-1], out=gstarts[1:])
    rgb_s = RGB[order]

    med6 = group_stat(inv, len(uniq), n, RGB, K_RING)
    lumA = lum_rgb3(med6)

    tb = dict(uniq=uniq, inv=inv, n=n,
              cent=np.stack([np.bincount(inv, weights=Pw[:, c], minlength=len(uniq)) / n
                             for c in range(3)], 1))
    multi = np.where(n >= 2)[0]
    c0, nrm, ok = fit_planes(uniq, tb["cent"], n, multi)
    d_signed = np.zeros(len(multi), object)
    if len(multi):
        Pv = Pw[order]
        for gi, g in enumerate(multi):
            r0, nn = gstarts[g], n[g]
            d_signed[gi] = ((Pv[r0:r0 + nn] - c0[gi]) @ nrm[gi]) if ok[gi] else np.zeros(nn)
    final, gstats = run_G(n, rgb_s, gstarts, d_signed, ok, multi, D_THR, C_THR)
    lumG = lum_rgb3(final)

    cells, covsum = raster_eval(Pw, RGB, fr, starts, poses, cam_params(),
                                lumA, lumG, n, uniq)
    geo = geo_metrics(Pw, fr, n, uniq, inv, gstarts, order)
    return dict(name=name, src=src, mode=mode, obs=int(len(Pw)),
                covsum=int(covsum), geo=geo, pose=pstats, cells=cells,
                g_accept=gstats["accepted"] / max(gstats["subsequent"], 1),
                g_reject=gstats["rejected"] / max(gstats["subsequent"], 1),
                multi_voxels=int(len(multi)))


def main():
    t0 = time.time()
    os.makedirs(OUT, exist_ok=True)
    sl_pool = pooled_sl()
    cache = {}
    for src in ["sl", "e1", "fl2"]:
        cache[src] = load_source(src)
        print(f"src {src}: {len(cache[src][0])} 帧, odom {len(cache[src][1])}", flush=True)
    levels = [dict(name="sl_clean", src="sl", mode="clean", pos=0.0, rot_deg=0.0),
              dict(name="drift_3cm", src="sl", mode="drift", pos=0.03, rot_deg=0.12),
              dict(name="drift_10cm", src="sl", mode="drift", pos=0.10, rot_deg=0.4),
              dict(name="drift_25cm", src="sl", mode="drift", pos=0.25, rot_deg=1.0),
              dict(name="jit_3cm", src="sl", mode="jit", pos=0.03, rot_deg=0.12),
              dict(name="jit_10cm", src="sl", mode="jit", pos=0.10, rot_deg=0.4),
              dict(name="jit_25cm", src="sl", mode="jit", pos=0.25, rot_deg=1.0),
              dict(name="e1_real", src="e1", mode="clean", pos=0.0, rot_deg=0.0)]
    # fl2 排除：fl2_frames bag 是不同场景窗口（路径 121.5m / z 跨 6.4m，
    # sl/e1 为同一段 10m 室内窗口），且未配置其相机外参，栅格不可比。
    # 已实测：fl2 级栅格 21.57% 系场景错配伪值。
    rows = []
    for lv in levels:
        try:
            r = build_level(**lv, sl_pool=sl_pool, cache=cache)
        except Exception as e:
            print(f"[{lv['name']}] 失败: {type(e).__name__}: {e}", flush=True)
            import traceback; traceback.print_exc()
            continue
        rows.append(r)
        with open(os.path.join(OUT, "summary.json"), "w") as f:
            json.dump(dict(runtime_s=time.time() - t0, levels=rows), f,
                      ensure_ascii=False, indent=1)
        c = r["cells"]; g = r["geo"]
        print(f"[{r['name']}] obs={r['obs']:,} 椒盐 F={c['F']['rate']:.2f}% "
              f"A={c['A']['rate']:.2f}% G={c['G']['rate']:.2f}% | "
              f"碰撞={g['collide_frac']*100:.1f}% spread中位={g['spread_med_cm']:.2f}cm "
              f"plane_rms={g['plane_rms_cm']:.2f}cm "
              f"pose_dev={r['pose']['pose_dev_mean_cm']:.1f}cm "
              f"相邻帧Δt={r['pose']['rel_trans_mean_cm']:.1f}cm "
              f"({time.time()-t0:.0f}s)", flush=True)
    with open(os.path.join(OUT, "summary.json"), "w") as f:
        json.dump(dict(runtime_s=time.time() - t0, levels=rows), f,
                  ensure_ascii=False, indent=1)
    print(f"完成 {time.time()-t0:.0f}s -> {OUT}/summary.json")


if __name__ == "__main__":
    main()
