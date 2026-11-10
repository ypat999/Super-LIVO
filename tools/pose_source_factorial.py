#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""位姿来源 × 着色模型 全交叉实验（用户 2026-11-09 指定设计，纯离线）。

约束（用户原话）：所有组合必须使用同一批几何点、同一组图像颜色、同一投影
模型、同一渲染掩码——只换位姿来源。

彩色场景（Retail 22s 段）无真 GT（自制数据）。本脚本用 **PLC 位姿图离线
精化**构造"伪 GT"：在该数据集上几何最自洽（差模误差最小）的轨迹。
残差椒盐 = 位姿无关地板的上界；sl 率 − 精化率 ≈ 位姿可归因部分。

位姿来源：
  sl       —— 基线（=§14 sl_clean，回归对照）
  e1       —— 真实另一次 Super-LIO 跑（与 sl 同世界系，直接替换）
  refined  —— PLC(point-to-plane/line/position) 高斯-牛顿多帧精化（伪 GT）
着色模型：F 逐观测自身色 / A K=6 中位数 / G 一致性门控(5mm,40)。

附加输出：各轨迹的帧间 NN blur（模型无关的几何一致性度量，lag-2 帧对）、
相对 sl 的帧对平移差（放到 §14 剂量轴上定位）。
"""
import json
import os
import sys
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from blackframe_voxel_replay import (load_bag, make_interpolator, rgb_of,
                                     group_stat, lum_rgb3, pack_i64, QMEM)
from voxel_attribution import load_obs, voxel_table, pack16, OBS, VOX
from voxel_fusion_G import fit_planes, run_G, JUMP_THR, K_RING
from lio_color_factorial import pooled_sl, raster_eval, geo_metrics, cam_params

OUT = "/home/ywj/slam_data/singleframe/blackframe_replay/pose_source_factorial"
BAG_SL = "/home/ywj/slam_data/singleframe/sl_frames"
BAG_E1 = "/home/ywj/slam_data/e1_append/e1_frames"


# ---------------------------------------------------------------- PLC 精化
def _line_basis(d):
    k = np.abs(d).argmin(1)
    e = np.eye(3)[k]
    u1 = np.cross(d, e)
    u1 /= np.maximum(np.linalg.norm(u1, axis=1, keepdims=True), 1e-9)
    u2 = np.cross(d, u1)
    return np.stack([u1, u2], 2)                       # (m,3,2)


def _cov3(SS6, mu, n):
    pairs = ((0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2))
    out = np.zeros((len(mu), 3, 3))
    for k, (a, b) in enumerate(pairs):
        v = SS6[:, k] / np.maximum(n, 1) - mu[:, a] * mu[:, b]
        out[:, a, b] = v
        out[:, b, a] = v
    return out


def _map_stats(Pws, sizes, nf, edge):
    cat = np.concatenate(Pws)
    farr = np.repeat(np.arange(nf, dtype=np.int64), sizes)
    vox = np.floor(cat / edge).astype(np.int32)
    key = pack_i64(vox, 21)
    uq, gi = np.unique(key, return_inverse=True)
    m = len(uq)
    fk = gi * nf + farr
    cnt = np.bincount(fk, minlength=m * nf).astype(np.float64).reshape(m, nf)
    sm = np.stack([np.bincount(fk, weights=cat[:, c], minlength=m * nf).reshape(m, nf)
                   for c in range(3)], 2)
    SS = np.stack([np.bincount(fk, weights=cat[:, a] * cat[:, b], minlength=m * nf).reshape(m, nf)
                   for a, b in ((0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2))], 2)
    return gi, farr, cnt.sum(1), sm.sum(1), SS.sum(1), cnt, sm, SS, m


def _loo_frame_params(i, n_all, s_all, SS_all, cnt, sm, SS):
    """其他帧构成的体素参数（LOO），返回 lut、mu、nrm、U、plane/line 掩码。"""
    n_o = n_all - cnt[:, i]
    s_o = s_all - sm[:, i]
    SS_o = SS_all - SS[:, i]
    vidx = np.where(n_o >= 4)[0]
    mu_k = s_o[vidx] / n_o[vidx, None]
    C_k = _cov3(SS_o[vidx], mu_k, n_o[vidx])
    evals, evecs = np.linalg.eigh(C_k)
    tot = np.maximum(evals.sum(1), 1e-12)
    sP = evals[:, 1] - evals[:, 0]
    sL = evals[:, 2] - evals[:, 1]
    plane = (sP > 0.15 * tot) & (sP > 3 * np.maximum(sL, 1e-12))
    line = (~plane) & (sL > 0.15 * tot) & (sL > 3 * np.maximum(sP, 1e-12))
    lut = np.full(n_all.shape[0], -1, np.int64)
    lut[vidx] = np.arange(len(vidx))
    return lut, mu_k, evecs[:, :, 0], _line_basis(evecs[:, :, 2]), plane, line


def _cost_and_terms(Pw, gi_i, lut, mu_k, nrm_k, U_k, plane_k, line_k, want_terms):
    loc = lut[gi_i]
    sel = loc >= 0
    if not sel.any():
        return 0.0, None
    li = loc[sel]; PP = Pw[sel]; dd = PP - mu_k[li]
    pl = plane_k[li]; ln = line_k[li]; ps = ~(pl | ln)
    rp = np.abs(np.einsum("ij,ij->i", nrm_k[li], dd)) if pl.any() else np.zeros(0)
    rl = np.linalg.norm(np.einsum("pjk,pj->pk", U_k[li], dd), axis=1) if ln.any() else np.zeros(0)
    rq = np.linalg.norm(dd, axis=1) if ps.any() else np.zeros(0)
    cost = (np.minimum(rp, 0.4).sum() if pl.any() else 0.0) + \
           (np.minimum(rl, 0.4).sum() if ln.any() else 0.0) + \
           (np.minimum(rq, 0.4).sum() if ps.any() else 0.0)
    if not want_terms:
        return float(cost), None
    terms = (PP, dd, li, pl, ln, ps)
    return float(cost), terms


def refine_poses(frames, pose_init, iters=10, edge=0.25, cauchy=0.3,
                 clip_t=0.05, clip_r=0.02, verbose=True):
    """LOO-PLC 位姿图精化：其他帧建图→残差→GN+线搜索。frame0 锚定。"""
    Pb = [P.astype(np.float64) for (_t, P, _pk) in frames]
    sizes = [len(P) for P in Pb]
    nf = len(Pb)
    Rs = np.array([R.copy() for R, _t in pose_init])
    tv = np.array([t.copy() for _R, t in pose_init])
    hist = []
    prev_cost = None
    for it in range(iters):
        Pws = [Pb[i] @ Rs[i].T + tv[i] for i in range(nf)]
        gi, farr, n_all, s_all, SS_all, cnt, sm, SS, m = _map_stats(Pws, sizes, nf, edge)
        ordby = np.argsort(farr, kind="stable")
        fstarts = np.zeros(nf + 1, np.int64)
        np.cumsum(np.bincount(farr, minlength=nf), out=fstarts[1:])
        total = 0.0
        for i in range(nf):
            lut, mu_k, nrm_k, U_k, plane_k, line_k = _loo_frame_params(
                i, n_all, s_all, SS_all, cnt, sm, SS)
            gi_i = gi[ordby[fstarts[i]:fstarts[i + 1]]]
            base, terms = _cost_and_terms(Pws[i], gi_i, lut, mu_k, nrm_k, U_k,
                                          plane_k, line_k, want_terms=True)
            total += base
            if terms is None or i == 0:
                continue
            PP, dd, li, pl, ln, ps = terms
            H = np.zeros((6, 6)); B = np.zeros(6)

            def _wb(r, A):
                dn = np.linalg.norm(r, axis=1)
                w = 1.0 / (1.0 + (dn / cauchy) ** 2)
                w[dn > 1.0] = 0.0
                return (np.einsum("m,mkj,mkl->jl", w, A, A),
                        np.einsum("m,mk,mkj->j", w, r, A))

            if pl.any():
                nn = nrm_k[li[pl]]
                r = np.einsum("ij,ij->i", nn, dd[pl])[:, None]
                A = np.concatenate([nn, -np.cross(nn, PP[pl])], 1)[:, None, :]
                HI, BI = _wb(r, A); H += HI; B += BI
            if ln.any():
                Ui = U_k[li[ln]]; PP2 = PP[ln]
                r = np.einsum("pmk,pm->pk", Ui, dd[ln])
                px = np.zeros((len(PP2), 3, 3))
                px[:, 0, 1] = -PP2[:, 2]; px[:, 0, 2] = PP2[:, 1]
                px[:, 1, 0] = PP2[:, 2]; px[:, 1, 2] = -PP2[:, 0]
                px[:, 2, 0] = -PP2[:, 1]; px[:, 2, 1] = PP2[:, 0]
                A = np.concatenate([Ui.transpose(0, 2, 1),
                                    -np.einsum("pma,pmj->paj", Ui, px)], 2)
                HI, BI = _wb(r, A); H += HI; B += BI
            if ps.any():
                PP2 = PP[ps]
                px = np.zeros((len(PP2), 3, 3))
                px[:, 0, 1] = -PP2[:, 2]; px[:, 0, 2] = PP2[:, 1]
                px[:, 1, 0] = PP2[:, 2]; px[:, 1, 2] = -PP2[:, 0]
                px[:, 2, 0] = -PP2[:, 1]; px[:, 2, 1] = PP2[:, 0]
                A = np.concatenate([np.broadcast_to(np.eye(3), (len(PP2), 3, 3)), -px], 2)
                HI, BI = _wb(dd[ps], A); H += HI; B += BI
            try:
                delta = np.linalg.solve(H + np.eye(6) * 1e-8 * max(np.trace(H) / 6, 1.0), -B)
            except np.linalg.LinAlgError:
                continue
            dv, dw = delta[:3], delta[3:]
            nv, nr = np.linalg.norm(dv), np.linalg.norm(dw)
            if nv > clip_t:
                dv *= clip_t / nv
            if nr > clip_r:
                dw *= clip_r / nr
            accepted = False
            for alpha in (1.0, 0.5, 0.25):
                Rc = Rotation.from_rotvec(alpha * dw).as_matrix() @ Rs[i]
                tc = tv[i] + alpha * dv
                Pc = Pb[i] @ Rc.T + tc
                c2, _ = _cost_and_terms(Pc, gi_i, lut, mu_k, nrm_k, U_k,
                                        plane_k, line_k, want_terms=False)
                if c2 <= base + 1e-6:
                    Rs[i], tv[i] = Rc, tc
                    total += (c2 - base)
                    accepted = True
                    break
            if not accepted:
                pass
        hist.append(dict(iter=it, cost=total, npoints=int(sum(sizes))))
        if verbose:
            print(f"  refine it{it}: cost={total:,.0f} voxels={m:,}", flush=True)
        if prev_cost is not None and abs(prev_cost - total) < 0.005 * abs(prev_cost):
            break
        prev_cost = total
    return [(Rs[i], tv[i]) for i in range(nf)], hist


def consistency_cost(frames, poses, edge=0.25):
    """LOO 稳健残差代价 /点（cm）：位姿间几何一致性度量（与颜色无关）。"""
    Pb = [P.astype(np.float64) for (_t, P, _pk) in frames]
    sizes = [len(P) for P in Pb]
    nf = len(Pb)
    Pws = [Pb[i] @ poses[i][0].T + poses[i][1] for i in range(nf)]
    gi, farr, n_all, s_all, SS_all, cnt, sm, SS, m = _map_stats(Pws, sizes, nf, edge)
    ordby = np.argsort(farr, kind="stable")
    fstarts = np.zeros(nf + 1, np.int64)
    np.cumsum(np.bincount(farr, minlength=nf), out=fstarts[1:])
    tot = 0.0; cov = 0
    for i in range(nf):
        lut, mu_k, nrm_k, U_k, plane_k, line_k = _loo_frame_params(
            i, n_all, s_all, SS_all, cnt, sm, SS)
        gi_i = gi[ordby[fstarts[i]:fstarts[i + 1]]]
        c, _ = _cost_and_terms(Pws[i], gi_i, lut, mu_k, nrm_k, U_k,
                               plane_k, line_k, want_terms=False)
        tot += c
        cov += (lut[gi_i] >= 0).sum()
    return dict(cost_per_pt_cm=float(tot / max(cov, 1) * 100),
                cov_frac=float(cov / sum(sizes)))


# ---------------------------------------------------------------- blur 度量
def blur_metric(Pws, rng, lag=2, nq=3000):
    ds = []
    for i in range(0, len(Pws) - lag, max(1, (len(Pws) - lag) // 6)):
        Pi, Pj = Pws[i], Pws[i + lag]
        qi = Pi[rng.choice(len(Pi), min(nq, len(Pi)), replace=False)]
        d = cKDTree(Pj).query(qi)[0]
        ds.append(float(np.median(d)))
    return float(np.mean(ds))


# ---------------------------------------------------------------- 主流程
def e1_side_experiment():
    """e1 几何基准的三源位姿交叉：{e1 自身, sl odom, e1 自精化} × {F,A,G}。
    目的：§14 e1_real=18.8% 的位姿归因检验——同几何换位姿来源。"""
    t0 = time.time()
    os.makedirs(OUT, exist_ok=True)
    MAXF = 40
    frames_e, odom_e = load_bag(BAG_E1)
    idx = np.linspace(0, len(frames_e) - 1, MAXF).astype(int)
    frames = [frames_e[i] for i in idx]
    pose_at_e = make_interpolator(odom_e)
    frames_s, odom_s = load_bag(BAG_SL)
    pose_at_s = make_interpolator(odom_s)
    pose_e = [pose_at_e(t) for (t, _P, _pk) in frames]
    pose_s = [pose_at_s(t) for (t, _P, _pk) in frames]
    print("== refine e1 (self)", flush=True)
    pose_ref, hist = refine_poses(frames, pose_e)

    Pb = [P.astype(np.float64) for (_t, P, _pk) in frames]
    sizes = [len(P) for P in Pb]
    cons = {"e1_own": consistency_cost(frames, pose_e),
            "sl_odom": consistency_cost(frames, pose_s),
            "e1_refined": consistency_cost(frames, pose_ref)}
    for k, v in cons.items():
        print(f"  consistency[{k}] {v['cost_per_pt_cm']:.2f}cm/pt "
              f"cov={v['cov_frac']*100:.0f}%", flush=True)

    cam = cam_params()
    fr_l, Pb_l, C_l = [], [], []
    for fi in range(len(frames)):
        (_t, P, pk) = frames[fi]
        keep = pk != 0
        fr_l.append(np.full(int(keep.sum()), fi, np.int32))
        Pb_l.append(P[keep].astype(np.float64))
        C_l.append(rgb_of(pk[keep]))
    fr0 = np.concatenate(fr_l)
    RGB0 = np.concatenate(C_l).astype(np.float64)
    nk = np.array([len(p) for p in Pb_l], np.int64)
    starts = np.zeros(len(frames) + 1, np.int64)
    np.cumsum(nk, out=starts[1:])

    results = {}
    for nm, poses in [("e1_own", pose_e), ("sl_odom", pose_s),
                      ("e1_refined", pose_ref)]:
        Pw = np.empty_like(RGB0)
        pos = 0
        for fi in range(len(frames)):
            nfi = len(Pb_l[fi])
            Pw[pos:pos + nfi] = Pb_l[fi] @ poses[fi][0].T + poses[fi][1]
            pos += nfi
        vox = np.floor(Pw / VOX).astype(np.int32)
        uniq, inv = np.unique(pack16(vox), return_inverse=True)
        n = np.bincount(inv, minlength=len(uniq))
        order = np.argsort(inv, kind="stable")
        gstarts = np.zeros(len(uniq), np.int64)
        np.cumsum(n[:-1], out=gstarts[1:])
        rgb_s = RGB0[order]
        med6 = group_stat(inv, len(uniq), n, RGB0, K_RING)
        lumA = lum_rgb3(med6)
        cent = np.stack([np.bincount(inv, weights=Pw[:, c], minlength=len(uniq)) / n
                         for c in range(3)], 1)
        multi = np.where(n >= 2)[0]
        c0, nrm, ok = fit_planes(uniq, cent, n, multi)
        Pv = Pw[order]
        d_signed = np.zeros(len(multi), object)
        for gi_, g in enumerate(multi):
            r0, nn = gstarts[g], n[g]
            d_signed[gi_] = ((Pv[r0:r0 + nn] - c0[gi_]) @ nrm[gi_]) if ok[gi_] else np.zeros(nn)
        final, gstats = run_G(n, rgb_s, gstarts, d_signed, ok, multi, 0.005, 40)
        lumG = lum_rgb3(final)
        cells, covsum = raster_eval(Pw, RGB0, fr0, starts, poses, cam,
                                    lumA, lumG, n, uniq)
        geo = geo_metrics(Pw, fr0, n, uniq, inv, gstarts, order)
        results[nm] = dict(obs=int(len(Pw)), covsum=int(covsum),
                           cells={k: v["rate"] for k, v in cells.items()},
                           cells_px={k: v["pixels"] for k, v in cells.items()},
                           geo=geo,
                           g_accept=gstats["accepted"] / max(gstats["subsequent"], 1))
        print(f"[e1geom×{nm}] obs={results[nm]['obs']:,} "
              f"F={results[nm]['cells']['F']:.2f}% A={results[nm]['cells']['A']:.2f}% "
              f"G={results[nm]['cells']['G']:.2f}% "
              f"({time.time()-t0:.0f}s)", flush=True)

    # ---- 密度抽稀对照：e1 几何+e1 位姿，逐帧随机保留 ~12k 点（对齐 sl 池密度）
    rng2 = np.random.default_rng(11)
    sel = []
    for fi in range(len(frames)):
        nfi = len(Pb_l[fi])
        take = min(12000, nfi)
        s = np.sort(rng2.choice(nfi, take, replace=False))
        sel.append(s + int(starts[fi]))
    selc = np.concatenate(sel)
    fr_t = fr0[selc]
    RGB_t = RGB0[selc]
    starts_t = np.zeros(len(frames) + 1, np.int64)
    np.add.at(starts_t[1:], fr_t, 1)
    np.cumsum(starts_t[1:], out=starts_t[1:])
    Pb_sel = [selc[starts_t[fi]:starts_t[fi + 1]] - starts[fi] for fi in range(len(frames))]
    Pw_t = np.empty_like(RGB_t)
    pos = 0
    for fi in range(len(frames)):
        gidx = Pb_sel[fi]
        Pw_t[pos:pos + len(gidx)] = Pb_l[fi][gidx] @ pose_e[fi][0].T + pose_e[fi][1]
        pos += len(gidx)
    vox = np.floor(Pw_t / VOX).astype(np.int32)
    uniq, inv = np.unique(pack16(vox), return_inverse=True)
    n = np.bincount(inv, minlength=len(uniq))
    order = np.argsort(inv, kind="stable")
    gstarts = np.zeros(len(uniq), np.int64)
    np.cumsum(n[:-1], out=gstarts[1:])
    med6 = group_stat(inv, len(uniq), n, RGB_t, K_RING)
    lumA = lum_rgb3(med6)
    cent = np.stack([np.bincount(inv, weights=Pw_t[:, c], minlength=len(uniq)) / n
                     for c in range(3)], 1)
    multi = np.where(n >= 2)[0]
    c0, nrm, ok = fit_planes(uniq, cent, n, multi)
    Pv = Pw_t[order]
    d_signed = np.zeros(len(multi), object)
    for gi_, g in enumerate(multi):
        r0, nn = gstarts[g], n[g]
        d_signed[gi_] = ((Pv[r0:r0 + nn] - c0[gi_]) @ nrm[gi_]) if ok[gi_] else np.zeros(nn)
    final, gstats = run_G(n, RGB_t[order], gstarts, d_signed, ok, multi, 0.005, 40)
    lumG = lum_rgb3(final)
    cells, covsum = raster_eval(Pw_t, RGB_t, fr_t, starts_t, pose_e, cam,
                                lumA, lumG, n, uniq)
    results["e1_own_thinned12k"] = dict(
        obs=int(len(Pw_t)), covsum=int(covsum),
        cells={k: v["rate"] for k, v in cells.items()},
        cells_px={k: v["pixels"] for k, v in cells.items()})
    print(f"[e1geom×e1_own_thinned] obs={len(Pw_t):,} "
          f"F={results['e1_own_thinned12k']['cells']['F']:.2f}% "
          f"A={results['e1_own_thinned12k']['cells']['A']:.2f}% "
          f"G={results['e1_own_thinned12k']['cells']['G']:.2f}% "
          f"({time.time()-t0:.0f}s)", flush=True)

    out = dict(kind="e1_side", runtime_s=time.time() - t0, consistency=cons,
               frames=[int(f[0]) for f in frames[:3]] + ["..."], levels=results,
               refine_hist=hist)
    with open(os.path.join(OUT, "summary_e1_side.json"), "w") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print("完成", flush=True)


def main():
    t0 = time.time()
    os.makedirs(OUT, exist_ok=True)
    if "e1only" in sys.argv:
        return e1_side_experiment()
    rng = np.random.default_rng(7)
    frames_sl, odom_sl = load_bag(BAG_SL)
    frames_e1, odom_e1 = load_bag(BAG_E1)
    pose_at_sl = make_interpolator(odom_sl)
    pose_at_e1 = make_interpolator(odom_e1)
    pose_init = [pose_at_sl(t) for (t, _P, _pk) in frames_sl]
    poses_e1 = [pose_at_e1(t) for (t, _P, _pk) in frames_sl]   # 按 sl 帧时刻取 e1 位姿

    # ---- PLC 精化（伪 GT） ----
    print("== PLC refine", flush=True)
    poses_ref, hist = refine_poses(frames_sl, pose_init)

    def world_of(poses):
        return [P.astype(np.float64) @ R.T + tv
                for (R, tv), (_t, P, _pk) in zip(poses, frames_sl)]
    Wsl, We1, Wrf = world_of(pose_init), world_of(poses_e1), world_of(poses_ref)
    blur = {"sl": blur_metric(Wsl, rng), "e1": blur_metric(We1, rng),
            "refined": blur_metric(Wrf, rng)}
    print("== consistency (LOO robust residual)", flush=True)
    cons = {"sl": consistency_cost(frames_sl, pose_init),
            "e1": consistency_cost(frames_sl, poses_e1),
            "refined": consistency_cost(frames_sl, poses_ref)}
    for k in cons:
        print(f"  {k}: {cons[k]['cost_per_pt_cm']:.2f}cm/pt "
              f"cov={cons[k]['cov_frac']*100:.0f}%", flush=True)
    dev = {}
    for nm, plist in [("e1", poses_e1), ("refined", poses_ref)]:
        dv = np.array([p[1] - q[1] for p, q in zip(plist, pose_init)])
        pw = np.linalg.norm(dv[:, None, :] - dv[None, :, :], axis=2)
        iu_ = np.triu_indices(len(plist), 1)
        dr = np.array([Rotation.from_matrix(p[0] @ q[0].T).as_rotvec()
                       for p, q in zip(plist, pose_init)])
        pwr = np.linalg.norm(dr[:, None, :] - dr[None, :, :], axis=2)
        dev[nm] = dict(dev_vs_sl_mean_cm=float(np.linalg.norm(dv, axis=1).mean() * 100),
                       pairwise_trans_cm=float(pw[iu_].mean() * 100),
                       pairwise_rot_deg=float(np.rad2deg(pwr[iu_].mean())))

    # ---- 观测集（固定：sl CSV pool + 体系恢复） ----
    fr0, P0, RGB0 = pooled_sl()
    Pb_pool = np.empty_like(P0)
    for fi in range(len(frames_sl)):
        m = fr0 == fi
        if m.any():
            R, tv = pose_init[fi]
            Pb_pool[m] = (P0[m] - tv) @ R

    frs = np.unique(fr0)
    starts = np.zeros(len(frames_sl) + 1, np.int64)
    np.add.at(starts[1:], fr0, 1)
    np.cumsum(starts[1:], out=starts[1:])

    cam = cam_params()
    results = {}
    for src_name, poses in [("sl", pose_init), ("e1", poses_e1), ("refined", poses_ref)]:
        Pw = np.empty_like(P0)
        for fi in range(len(frames_sl)):
            m = fr0 == fi
            if m.any():
                R, tv = poses[fi]
                Pw[m] = Pb_pool[m] @ R.T + tv
        vox = np.floor(Pw / VOX).astype(np.int32)
        uniq, inv = np.unique(pack16(vox), return_inverse=True)
        n = np.bincount(inv, minlength=len(uniq))
        order = np.argsort(inv, kind="stable")
        gstarts = np.zeros(len(uniq), np.int64)
        np.cumsum(n[:-1], out=gstarts[1:])
        RGB = RGB0.astype(np.float64)
        rgb_s = RGB[order]
        med6 = group_stat(inv, len(uniq), n, RGB, K_RING)
        lumA = lum_rgb3(med6)
        cent = np.stack([np.bincount(inv, weights=Pw[:, c], minlength=len(uniq)) / n
                         for c in range(3)], 1)
        multi = np.where(n >= 2)[0]
        c0, nrm, ok = fit_planes(uniq, cent, n, multi)
        Pv = Pw[order]
        d_signed = np.zeros(len(multi), object)
        for gi_, g in enumerate(multi):
            r0, nn = gstarts[g], n[g]
            d_signed[gi_] = ((Pv[r0:r0 + nn] - c0[gi_]) @ nrm[gi_]) if ok[gi_] else np.zeros(nn)
        final, gstats = run_G(n, rgb_s, gstarts, d_signed, ok, multi, 0.005, 40)
        lumG = lum_rgb3(final)
        cells, covsum = raster_eval(Pw, RGB, fr0, starts, poses, cam,
                                    lumA, lumG, n, uniq)
        geo = geo_metrics(Pw, fr0, n, uniq, inv, gstarts, order)
        results[src_name] = dict(
            cells={k: v["rate"] for k, v in cells.items()},
            cells_px={k: v["pixels"] for k, v in cells.items()},
            covsum=int(covsum), geo=geo,
            collide_frac=geo["collide_frac"], spread_med_cm=geo["spread_med_cm"],
            plane_rms_cm=geo["plane_rms_cm"],
            g_accept=gstats["accepted"] / max(gstats["subsequent"], 1))
        print(f"[{src_name}] F={results[src_name]['cells']['F']:.2f}% "
              f"A={results[src_name]['cells']['A']:.2f}% "
              f"G={results[src_name]['cells']['G']:.2f}% blur={blur[src_name]*100:.2f}cm "
              f"pw_vs_sl={dev.get(src_name,{}).get('pairwise_trans_cm',0):.1f}cm "
              f"({time.time()-t0:.0f}s)", flush=True)

    out = dict(runtime_s=time.time() - t0, blur_cm={k: v * 100 for k, v in blur.items()},
               consistency=cons,
               pose_dev_vs_sl=dev, refine_hist=hist, levels=results,
               dose_curve_pred=dict(
                   e1_pred_pp=6.11 + 0.244 * dev["e1"]["pairwise_trans_cm"]))
    with open(os.path.join(OUT, "summary.json"), "w") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    np.savez(os.path.join(OUT, "poses_refined.npz"),
             R=np.stack([p[0] for p in poses_ref]), t=np.stack([p[1] for p in poses_ref]),
             R_sl=np.stack([p[0] for p in pose_init]), t_sl=np.stack([p[1] for p in pose_init]))
    print(f"完成 {time.time()-t0:.0f}s -> {OUT}/summary.json")


if __name__ == "__main__":
    main()
