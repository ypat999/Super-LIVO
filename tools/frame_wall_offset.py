#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""逐帧位姿误差序列分析（纯离线）。

输入：Super-LIO 逐帧导出的 scans_*.pcd —— 这是 **OctVox 之前** 的数据
（DenseThread：全量去畸变 → delta correction → 用该帧修正后位姿 T_corrected
变换到世界系 → 纯 append 到 point_map_，每帧清空），所以这里看到的重影
与地图融合策略无关。scans_*.txt 给出该帧位姿（时间戳 + 位置 + 四元数）。

为什么不用"几面大墙"做刚体检验：本场景只能分离出 2 个大平面法向（街道两侧
是平行墙，被法向去重合并），每帧 6 个未知数因此**欠定**（绕法向的旋转不可
辨识）：实测设计矩阵条件数 inf、留一平面 held-out R² 为负，样本内 R²=0.42
纯属过拟合零空间。改用**全场景局部平面体素**：每个体素自拟合平面，法向天然
覆盖地面/各朝向墙面/斜面/屋顶，数百体素 → 数百独立观测，6 未知数被强超定。

三个每帧指标（用户 2026-11-10 指定）：
  1. 法向偏移 d_k：各体素上"该帧平均深度 − 该体素时均深度"的幅度聚合
  2. 帧内厚度：该帧在该体素内绕自身均值的点散布（单帧几何一致性）
  3. 离群比例：该帧在该体素内偏离本帧均值 >3cm 的点占比
判据：
  A. 刚体可解释性：体素随机 50/50 划分，一半拟合每帧 6DoF、另一半做 held-out
     预测。只有 held-out R² > 0 才算证据（基线 0 = 假设无偏移）。
  B. 时间形态：对 δt/δθ 与 d_k 序列做 variogram V(m)=a+b·m；a 主导=逐帧独立
     抖动，b 主导=渐进随机游走漂移；另有连续帧跳变检测与 1D k-means 多峰检验。
  C. 与 odom 对照：恢复的 δt 与 odom 位置去趋势量（±W 帧滑动均值之差）相关。
"""
import argparse
import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame_color_from_scans import read_pcd_xyz
from voxel_attribution import pack16

IDX_RE = re.compile(r"scans_(\d+)\.pcd$")
PAIRS = ((0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2))


def load(src):
    files = []
    for fn in os.listdir(src):
        m = IDX_RE.match(fn)
        if m:
            files.append((int(m.group(1)), fn))
    files.sort()
    P, fr, pose = [], [], []
    for slot, (_, fn) in enumerate(files):
        p = read_pcd_xyz(os.path.join(src, fn))
        P.append(p)
        fr.append(np.full(len(p), slot, np.int32))
        row = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        tp = os.path.join(src, fn[:-4] + ".txt")
        if os.path.exists(tp):
            with open(tp) as f:
                v = f.read().split()
            if len(v) >= 8:
                row = [float(x) for x in v[:8]]
        pose.append(row)
        if (slot + 1) % 400 == 0:
            print(f"  loaded {slot+1}/{len(files)}")
    return np.concatenate(P).astype(np.float32), np.concatenate(fr), np.array(pose), [f for _, f in files]


# ------------------------------------------------- 体素局部平面 + 每帧深度
def voxel_planes(P, fr, nf, edge=0.5, min_pts=200, min_pf=20, min_frames=8,
                 planar_ratio=0.02, max_thick=0.04, out_thr=0.03,
                 max_vox=4000, seed=3):
    vox = np.floor(P / edge).astype(np.int32)
    uniq, inv = np.unique(pack16(vox), return_inverse=True)
    m = len(uniq)
    Pd = P.astype(np.float64)
    n = np.bincount(inv, minlength=m).astype(np.float64)
    cent = np.stack([np.bincount(inv, weights=Pd[:, c], minlength=m) for c in range(3)], 1) \
        / np.maximum(n[:, None], 1)
    cov = np.zeros((m, 3, 3))
    for k, (a, b) in enumerate(PAIRS):
        v = np.bincount(inv, weights=Pd[:, a] * Pd[:, b], minlength=m) / np.maximum(n, 1) \
            - cent[:, a] * cent[:, b]
        cov[:, a, b] = v
        cov[:, b, a] = v
    evals, evecs = np.linalg.eigh(cov)                    # 升序特征值
    ok = ((n >= min_pts)
          & (evals[:, 0] <= planar_ratio * np.maximum(evals[:, 2], 1e-12))
          & (np.sqrt(np.maximum(evals[:, 0], 0.0)) <= max_thick))
    sel = np.where(ok)[0]
    if len(sel) == 0:
        return None
    if len(sel) > max_vox:
        # (体素 × 帧) 稠密矩阵是 npat*nf，必须限规模；随机抽样不影响统计效力
        sel = np.sort(np.random.default_rng(seed).choice(sel, max_vox, replace=False))
    lut = np.full(m, -1, np.int64)
    lut[sel] = np.arange(len(sel))
    keep = lut[inv] >= 0
    pid = lut[inv][keep]
    F = fr[keep].astype(np.int64)
    nrm = evecs[sel][:, :, 0]
    c0 = cent[sel]
    d = np.einsum("ij,ij->i", nrm[pid], Pd[keep] - c0[pid])

    npat = len(sel)
    pk = pid * nf + F
    cnt = np.bincount(pk, minlength=npat * nf).reshape(npat, nf).astype(np.float64)
    s1 = np.bincount(pk, weights=d, minlength=npat * nf).reshape(npat, nf)
    s2 = np.bincount(pk, weights=d * d, minlength=npat * nf).reshape(npat, nf)
    good_pf = cnt >= min_pf
    mean = np.where(good_pf, s1 / np.maximum(cnt, 1), np.nan)
    within = np.where(good_pf,
                      np.sqrt(np.maximum(s2 / np.maximum(cnt, 1) - np.nan_to_num(mean) ** 2, 0.0)),
                      np.nan)
    nfr = good_pf.sum(1)
    good = nfr >= min_frames
    # 离群占比：|d − 该(体素,帧)均值| > out_thr
    dev = np.abs(d - np.nan_to_num(mean)[pid, F])
    oc = np.bincount(pk, weights=(dev > out_thr), minlength=npat * nf).reshape(npat, nf)
    out_share = np.where(good_pf, oc / np.maximum(cnt, 1), np.nan)
    # 按体素去掉时不变分量（该体素相对理想平面的静态偏移），只保留帧间变化
    w = np.where(good_pf, cnt, 0.0)
    mu = (np.nan_to_num(mean) * w).sum(1) / np.maximum(w.sum(1), 1.0)
    Y = mean - mu[:, None]
    return dict(cent=c0[good], normal=nrm[good], Y=Y[good], cnt=cnt[good],
                W=np.sqrt(np.where(np.isfinite(Y[good]), cnt[good], 0.0)),
                within=within[good], out_share=out_share[good],
                n_frames_obs=nfr[good], n_pts=cnt.sum(1)[good])


def design(vt, scene_c):
    lever = np.cross(vt["cent"] - scene_c, vt["normal"])
    return np.concatenate([vt["normal"], lever], 1)


# ------------------------------------------------- 每帧三指标聚合
def frame_aggregates(vt):
    Y, within, cnt, out_share = vt["Y"], vt["within"], vt["cnt"], vt["out_share"]
    obs = np.isfinite(Y)
    n_obs = obs.sum(0)
    den = np.maximum(n_obs, 1)
    mean_abs = np.where(n_obs > 0, np.where(obs, np.abs(Y), 0.0).sum(0) / den * 100, np.nan)
    med_abs = np.full(Y.shape[1], np.nan)
    for k in range(Y.shape[1]):
        v = Y[obs[:, k], k]
        if len(v) >= 20:
            med_abs[k] = np.median(np.abs(v)) * 100
    ok_w = np.isfinite(within)
    thick = np.where(ok_w.any(0), np.nanmedian(np.where(ok_w, within, np.nan), axis=0) * 100, np.nan)
    pw = np.where(np.isfinite(out_share), cnt, 0.0)
    num = np.where(np.isfinite(out_share), np.nan_to_num(out_share) * pw, 0.0).sum(0)
    dd = pw.sum(0)
    oshare = np.where(dd > 1000, num / np.maximum(dd, 1), np.nan)
    return dict(mean_abs_cm=mean_abs, median_abs_cm=med_abs, thickness_cm=thick,
                outlier_share=oshare, n_vox_obs=n_obs, valid=n_obs >= 50)


# ------------------------------------------------- 逐帧刚体拟合 + held-out
def rigid_per_frame(A, Y, W, fit_mask, pred_mask=None, min_obs=20):
    nf = Y.shape[1]
    sol = np.full((nf, 6), np.nan)
    r2_in = np.full(nf, np.nan)
    cond = np.full(nf, np.nan)
    rank = np.full(nf, np.nan)
    Af = A[fit_mask]
    Yf, Wf = Y[fit_mask], W[fit_mask]
    held = None
    if pred_mask is not None:
        Ap = A[pred_mask]
        Yp, Wp = Y[pred_mask], W[pred_mask]
        held = dict(r2_out=np.full(nf, np.nan), num=np.zeros(nf), den=np.zeros(nf))
    for k in range(nf):
        y, w = Yf[:, k], Wf[:, k]
        ok = np.isfinite(y) & (w > 0)
        if ok.sum() < min_obs:
            continue
        s, res, rk, sv = np.linalg.lstsq(Af[ok] * w[ok][:, None], y[ok] * w[ok], rcond=1e-8)
        sol[k, :len(s)] = s
        rank[k] = int(rk)
        cond[k] = float(sv[0] / sv[-1]) if len(sv) > 1 and sv[-1] > 1e-12 else np.inf
        r = y[ok] - Af[ok] @ s
        sst = float((w[ok] * y[ok] ** 2).sum())
        r2_in[k] = 1 - float((w[ok] * r ** 2).sum()) / sst if sst > 0 else np.nan
        if held is not None:
            yt, wt = Yp[:, k], Wp[:, k]
            ot = np.isfinite(yt) & (wt > 0)
            if ot.sum() >= min_obs:
                rt = yt[ot] - Ap[ot] @ s
                held["num"][k] = float((wt[ot] * rt ** 2).sum())
                held["den"][k] = float((wt[ot] * yt[ot] ** 2).sum())
    if held is not None:
        g = held["den"] > 0
        held["r2_out"][g] = 1 - held["num"][g] / held["den"][g]
        held["agg_r2"] = float(1 - held["num"][g].sum() / held["den"][g].sum()) if g.any() else None
        held["n_frames"] = int(g.sum())
    return dict(sol=sol, r2_in=r2_in, cond=cond, rank=rank, held=held)


# ------------------------------------------------- 时间形态
def variogram(d, valid, lags=(1, 2, 4, 8, 16, 32, 64, 128, 256)):
    ks = np.arange(len(d))[valid & np.isfinite(d)]
    dv = d[ks]
    pos = {int(k): i for i, k in enumerate(ks)}
    out = {}
    for m in lags:
        pr = [(i, pos[k + m]) for i, k in enumerate(ks) if (k + m) in pos]
        if len(pr) < 20:
            out[m] = (np.nan, len(pr))
            continue
        a = np.array([p[0] for p in pr])
        b = np.array([p[1] for p in pr])
        out[m] = (float(np.mean((dv[b] - dv[a]) ** 2)), len(pr))
    return out, ks, dv


def classify_temporal(d, valid, scale=100.0, unit="cm", seed=0):
    vg, ks, dv = variogram(d, valid)
    pts = sorted(((int(m), v[0], v[1]) for m, v in vg.items()
                  if np.isfinite(v[0]) and v[1] >= 20), key=lambda t: t[0])
    mm = np.array([p[0] for p in pts], float)
    vs = np.array([p[1] for p in pts], float) * scale ** 2
    fit = {}
    if len(mm) >= 3:
        A = np.stack([np.ones_like(mm), mm], 1)
        coef, *_ = np.linalg.lstsq(A, vs, rcond=None)
        a, b = float(coef[0]), float(coef[1])
        pr = A @ coef
        fit = dict(white_sigma=round(float(np.sqrt(max(a, 0))), 4),
                   rw_sigma_per_sqrtframe=round(float(np.sqrt(max(b, 0))), 4),
                   drift_share_at_max_lag=round(float(max(b, 0) * mm[-1] / max(a + b * mm[-1], 1e-30)), 3),
                   variogram_r2=round(float(1 - np.sum((vs - pr) ** 2) /
                                            max(np.sum((vs - vs.mean()) ** 2), 1e-30)), 3))
        fit["unit"] = unit
    dd, di = [], []
    for i in range(len(ks) - 1):
        if ks[i + 1] - ks[i] == 1:
            dd.append(dv[i + 1] - dv[i])
            di.append(i)
    dd = np.array(dd) * scale
    jumps = []
    if len(dd) > 10:
        med = float(np.median(dd))
        sc = float(1.4826 * np.median(np.abs(dd - med)))
        z = np.abs(dd - med) / max(sc, 1e-12)
        bad = np.where(z > 4)[0]
        jumps = [[int(ks[di[i]]), int(ks[di[i] + 1]), round(float(dd[i]), 4), round(float(z[i]), 1)]
                 for i in bad]
        fit["step_std"] = round(float(np.std(dd)), 4)
        fit["step_mad"] = round(sc, 4)
    x = dv.ravel() * scale
    gm = float(x.mean())
    sst = float(((x - gm) ** 2).sum())
    clus = {}
    for K in (2, 3):
        rng = np.random.default_rng(seed)
        cen = x[rng.choice(len(x), K, replace=False)].astype(float).copy()
        lab = np.full(len(x), -1, int)
        for _ in range(40):
            new = np.abs(x[:, None] - cen[None, :]).argmin(1)
            if (new == lab).all():
                break
            lab = new
            for q in range(K):
                if (lab == q).any():
                    cen[q] = x[lab == q].mean()
        mu = np.array([x[lab == q].mean() if (lab == q).any() else gm for q in range(K)])
        cnt = np.bincount(lab, minlength=K).astype(float)
        ssb = float((cnt * (mu - gm) ** 2).sum())
        # 零假设：打乱时间顺序后同一 k-means 的类间占比（单峰高斯也会给出非零值，
        # 只有显著高于该基线才算"在几个位置之间跳变"）
        xs = x[rng.permutation(len(x))]
        cen2 = xs[rng.choice(len(xs), K, replace=False)].astype(float).copy()
        lab2 = np.full(len(xs), -1, int)
        for _ in range(40):
            new2 = np.abs(xs[:, None] - cen2[None, :]).argmin(1)
            if (new2 == lab2).all():
                break
            lab2 = new2
            for q in range(K):
                if (lab2 == q).any():
                    cen2[q] = xs[lab2 == q].mean()
        mu2 = np.array([xs[lab2 == q].mean() if (lab2 == q).any() else gm for q in range(K)])
        cnt2 = np.bincount(lab2, minlength=K).astype(float)
        ssb2 = float((cnt2 * (mu2 - gm) ** 2).sum())
        clus[K] = dict(sep=round(float(mu.max() - mu.min()), 4),
                       between_share=round(float(ssb / sst), 3) if sst > 1e-12 else None,
                       null_between_share=round(float(ssb2 / sst), 3) if sst > 1e-12 else None,
                       counts=[int(c) for c in cnt])
    return dict(variogram={str(k): round(float(v[0]) * scale ** 2, 5)
                           for k, v in vg.items() if np.isfinite(v[0]) and v[1] >= 20},
                fit=fit, n_jump_frames=len(jumps), jumps=jumps[:15],
                clusters={str(k): v for k, v in clus.items()},
                std=round(float(np.std(dv) * scale), 4), unit=unit)


def odom_jitter(pose, W=5):
    """odom 的高频（抖动）分量：在 ±W 帧窗口内做**局部线性**拟合，取窗口中心处
    残差。不能用滑动均值——均值只去常数、不去斜率，会把匀速运动的真实位移当成
    "抖动"（W=20 时残差可达 0.5m，导致解释 R² 巨负）。
    位置：局部线性拟合在中心处的残差；旋转：以 R_k 为切空间基准，对窗口内
    rotvec 做局部线性拟合，取中心残差。返回 (nf,3) 平移抖动、(nf,3) 旋转抖动。"""
    from scipy.spatial.transform import Rotation
    pos, R = pose[:, 1:4], Rotation.from_quat(pose[:, 4:8]).as_matrix()
    n = len(pos)
    hp = np.zeros((n, 3))
    om = np.zeros((n, 3))
    for k in range(n):
        a, b = max(0, k - W), min(n, k + W + 1)
        if b - a < 4:
            continue
        t = np.arange(a, b, dtype=float) - k
        A = np.stack([np.ones_like(t), t], 1)
        coef, *_ = np.linalg.lstsq(A, pos[a:b], rcond=None)
        hp[k] = pos[k] - coef[0]                       # 中心处 t=0 的拟合值=截距
        v = Rotation.from_matrix(R[k].T @ R[a:b]).as_rotvec()
        coefr, *_ = np.linalg.lstsq(A, v, rcond=None)
        om[k] = -coefr[0]                              # 观测为 0，残差 = -截距
    return hp, om


def odom_explains(vt, A, hp, om):
    """良置检验：不做任何参数反演，直接用 odom 自己的高频位姿预测每个体素每帧的
    深度偏移 ŷ = n·δp + ((p−c)×n)·δθ，与实测 Y 比。基线为 0（Y 已按体素去均值），
    因此 R²>0 就表示"重影已经写在 odom 输出里"。"""
    Y = vt["Y"]
    out = {}
    for tag, dp, dr in (("pos+rot", hp, om), ("pos only", hp, np.zeros_like(om)),
                        ("rot only", np.zeros_like(hp), om)):
        num = den = 0.0
        for k in range(Y.shape[1]):
            y = Y[:, k]
            ok = np.isfinite(y)
            if ok.sum() < 20:
                continue
            d = np.concatenate([dr[k], dp[k]])
            r = y[ok] - A[ok] @ d
            num += float((r ** 2).sum())
            den += float((y[ok] ** 2).sum())
        out[tag] = round(float(1 - num / den), 4) if den > 0 else None
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="/home/ywj/slam_data/pcd/retail_framecolor/PCD")
    ap.add_argument("--out", default="/home/ywj/slam_data/pcd/retail_framecolor/walloff")
    ap.add_argument("--edge", type=float, default=0.5)
    ap.add_argument("--min-pts", type=int, default=200)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    P, fr, pose, names = load(a.src)
    nf = len(names)
    print(f"载入 {len(P)} 点 / {nf} 帧")

    vt = voxel_planes(P, fr, nf, edge=a.edge, min_pts=a.min_pts)
    if vt is None:
        print("无合格体素：放宽 --min-pts 或 --edge")
        return
    scene_c = P.mean(0).astype(np.float64)
    nv = vt["Y"].shape[0]
    A = design(vt, scene_c)
    M = A.T @ A
    ev = np.linalg.eigvalsh(M)
    print(f"局部平面体素 {nv} 个（edge={a.edge}m） 每体素观测帧数中位={int(np.median(vt['n_frames_obs']))}")
    print(f"可辨识性：设计 Gram 特征值 min={ev.min():.1f} max={ev.max():.1f} "
          f"cond={ev.max()/max(ev.min(),1e-12):.1f}（min>0 表示 6DoF 全可辨）")

    agg = frame_aggregates(vt)
    v = agg["valid"]
    print(f"\n每帧指标：|偏移|中位={np.nanmedian(agg['median_abs_cm'][v]):.2f}cm "
          f"帧内厚度中位={np.nanmedian(agg['thickness_cm'][v]):.2f}cm "
          f"离群占比中位={np.nanmedian(agg['outlier_share'][v]):.3f}")

    rng = np.random.default_rng(11)
    perm = rng.permutation(nv)
    Am = np.zeros(nv, bool)
    Am[perm[: nv // 2]] = True
    full = rigid_per_frame(A, vt["Y"], vt["W"], np.ones(nv, bool))
    ho = rigid_per_frame(A, vt["Y"], vt["W"], Am, ~Am)
    ho_out = ho["held"]["r2_out"]
    ho_t = rigid_per_frame(A[:, 3:], vt["Y"], vt["W"], Am, ~Am)   # 仅平移（3 参数）
    ho_r = rigid_per_frame(A[:, :3], vt["Y"], vt["W"], Am, ~Am)   # 仅旋转（3 参数）
    v6 = np.isfinite(full["r2_in"])
    vo = np.isfinite(ho_out)
    dt = full["sol"][:, 3:]
    dth = np.degrees(full["sol"][:, :3])
    hp = odom_jitter(pose, W=5)[0]
    corr_dt = [round(float(np.corrcoef(dt[v6][:, i], hp[v6][:, i])[0, 1]), 3) for i in range(3)]
    print("\n=== A. 刚体可解释性（体素 50/50 划分）===")
    print(f"  样本内 R² 中位={np.nanmedian(full['r2_in'][v6]):.3f} | "
          f"held-out R² 中位={np.nanmedian(ho_out[vo]):.3f} "
          f"聚合={ho['held']['agg_r2']:.3f}（有效帧 {ho['held']['n_frames']}）")
    print(f"  仅平移 held-out 聚合 R²={ho_t['held']['agg_r2']:.3f} | "
          f"仅旋转 held-out 聚合 R²={ho_r['held']['agg_r2']:.3f} | "
          f"6DoF={ho['held']['agg_r2']:.3f}")
    print(f"  逐帧可辨识自由度数 rank：中位={np.nanmedian(full['rank'][v6]):.0f}/6，"
          f"rank=6 的帧占 {100*np.mean(full['rank'][v6]==6):.0f}%（rank<6 时 δθ/δt 的"
          f"拆分不唯一，只有预测量可信）")
    print(f"  条件数中位={np.nanmedian(full['cond'][v6]):.1f}  "
          f"δt std={np.round(np.nanstd(dt[v6], axis=0)*100, 3).tolist()}cm  "
          f"δθ std={np.round(np.nanstd(dth[v6], axis=0), 3).tolist()}°")
    print(f"  δt 与 odom 高频相关={corr_dt}（注意：δθ/δt 单值受列相关性放大，"
          f"仅作参考；可信的是 held-out 预测力）")

    # C. 良置版：直接用 odom 自身的高频位姿预测偏移场，不做参数反演
    odom_r2 = {}
    for W in (5, 10, 20):
        jp, jr = odom_jitter(pose, W=W)
        odom_r2[f"W{W}"] = odom_explains(vt, A, jp, jr)
        print(f"  [C] odom 高频位姿(W=±{W}帧) 对实测偏移场的解释 R²: {odom_r2[f'W{W}']}")

    series = [("dt_x", dt[:, 0], 100.0, "cm"), ("dt_y", dt[:, 1], 100.0, "cm"),
              ("dt_z", dt[:, 2], 100.0, "cm"), ("dth_x", dth[:, 0], 1.0, "deg"),
              ("dth_y", dth[:, 1], 1.0, "deg"), ("dth_z", dth[:, 2], 1.0, "deg"),
              ("mean_abs_offset", agg["mean_abs_cm"], 1.0, "cm"),
              ("thickness", agg["thickness_cm"], 1.0, "cm"),
              ("outlier_share", agg["outlier_share"], 1.0, "ratio")]
    temporal = {}
    print("\n=== B. 时间形态（抖动σ=逐帧独立部分，游走σ=√帧随机游走部分）===")
    for key, arr, sc, un in series:
        vv = v & np.isfinite(arr)
        t = classify_temporal(arr, vv, scale=sc, unit=un)
        temporal[key] = t
        f = t["fit"]
        print(f"  {key:16s} std={t['std']:.4f}{un:5s} 抖动σ={f.get('white_sigma', float('nan')):.4f} "
              f"游走σ/√帧={f.get('rw_sigma_per_sqrtframe', float('nan')):.4f} "
              f"漂移占比={f.get('drift_share_at_max_lag', float('nan')):.2f} "
              f"跳变帧={t['n_jump_frames']:3d} 2类间隔={t['clusters']['2']['sep']:.4f}")

    summary = dict(n_points=int(len(P)), n_frames=nf, n_voxels=int(nv), voxel_edge_m=a.edge,
                   identifiability=dict(gram_eig_min=float(ev.min()), gram_cond=float(ev.max() / max(ev.min(), 1e-12))),
                   per_frame=dict(median_abs_offset_cm=round(float(np.nanmedian(agg["median_abs_cm"][v])), 3),
                                  thickness_cm=round(float(np.nanmedian(agg["thickness_cm"][v])), 3),
                                  outlier_share=round(float(np.nanmedian(agg["outlier_share"][v])), 4)),
                   rigid=dict(r2_in_med=round(float(np.nanmedian(full["r2_in"][v6])), 4),
                              r2_heldout_med=round(float(np.nanmedian(ho_out[vo])), 4),
                              r2_heldout_agg=round(ho["held"]["agg_r2"], 4),
                              r2_heldout_translation_only=round(ho_t["held"]["agg_r2"], 4),
                              r2_heldout_rotation_only=round(ho_r["held"]["agg_r2"], 4),
                              rank_med=float(np.nanmedian(full["rank"][v6])),
                              rank_full_share=round(float(np.mean(full["rank"][v6] == 6)), 3),
                              heldout_frames=ho["held"]["n_frames"],
                              cond_med=round(float(np.nanmedian(full["cond"][v6])), 1),
                              dt_std_cm=[round(float(x), 4) for x in np.nanstd(dt[v6], axis=0) * 100],
                              dtheta_std_deg=[round(float(x), 4) for x in np.nanstd(dth[v6], axis=0)],
                              dt_vs_odom_hp_corr=corr_dt),
                   odom_explains=odom_r2,
                   temporal=temporal)
    with open(os.path.join(a.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1, ensure_ascii=False, default=float)
    np.savez_compressed(os.path.join(a.out, "series.npz"),
                        mean_abs=agg["mean_abs_cm"], median_abs=agg["median_abs_cm"],
                        thickness=agg["thickness_cm"], outlier=agg["outlier_share"],
                        dt=dt, dtheta=dth, r2_in=full["r2_in"], r2_out=ho_out,
                        hp=hp, pose=pose, normals=vt["normal"], cent=vt["cent"])
    plot(agg, full, ho, hp, pose, dt, dth, v, nf, a.out)
    print(f"\n产物：{a.out}/summary.json series.npz series.png rigid.png")


def plot(agg, full, ho, hp, pose, dt, dth, v, nf, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"matplotlib 跳过：{e}")
        return
    k = np.arange(nf)

    fig, axs = plt.subplots(5, 1, figsize=(14, 14), sharex=True)
    axs[0].plot(k[v], agg["median_abs_cm"][v], ".-", lw=.6, ms=3)
    axs[0].set_ylabel("median |offset| (cm)")
    axs[0].set_title("1) per-frame normal offset")
    axs[1].plot(k[v], agg["thickness_cm"][v], ".-", lw=.6, ms=3, color="tab:orange")
    axs[1].set_ylabel("within-frame thickness (cm)")
    axs[1].set_title("2) single-frame geometry consistency")
    axs[2].plot(k[v], agg["outlier_share"][v], ".-", lw=.6, ms=3, color="tab:purple")
    axs[2].set_ylabel("outlier share")
    axs[2].set_title("3) per-frame outlier fraction")
    for i, lab in enumerate("xyz"):
        axs[3].plot(k[v], dt[v, i] * 100, lw=.7, label=f"dt_{lab}")
    axs[3].set_ylabel("recovered dt (cm)")
    axs[3].legend(fontsize=8, ncol=3)
    axs[3].set_title("rigid per-frame translation error")
    for i, lab in enumerate("xyz"):
        axs[4].plot(k[v], dth[v, i], lw=.7, label=f"dtheta_{lab}")
    axs[4].set_ylabel("recovered dtheta (deg)")
    axs[4].set_xlabel("frame index (time order)")
    axs[4].legend(fontsize=8, ncol=3)
    axs[4].set_title("rigid per-frame rotation error")
    for x in axs:
        x.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "series.png"), dpi=105)
    plt.close(fig)

    fig, axs = plt.subplots(1, 3, figsize=(17, 4.8))
    v6 = np.isfinite(full["r2_in"])
    axs[0].hist(full["r2_in"][v6], bins=30, alpha=.7, label="in-sample")
    ho_out = ho["held"]["r2_out"]
    axs[0].hist(ho_out[np.isfinite(ho_out)], bins=30, alpha=.7, label="held-out")
    axs[0].set_xlabel("R² per frame")
    axs[0].set_title("A: rigid explainability")
    axs[0].legend(fontsize=8)
    axs[0].axvline(0, color="k", lw=.8)
    axs[1].set_xscale("log"); axs[1].set_yscale("log")
    for i, (lab, c) in enumerate(zip("xyz", ["tab:blue", "tab:orange", "tab:green"])):
        vg, ks, dv = variogram(dt[:, i], v)
        ms = sorted(int(m) for m, (val, nn) in vg.items() if np.isfinite(val) and nn >= 20)
        axs[1].plot(ms, [vg[m][0] * 1e4 for m in ms], "o-", ms=4, color=c, label=f"dt_{lab}")
    axs[1].set_xlabel("lag (frames)"); axs[1].set_ylabel("V(m) cm$^2$")
    axs[1].set_title("B: variogram (flat=jitter, linear=drift)")
    axs[1].legend(fontsize=8); axs[1].grid(alpha=.3, which="both")
    for i in range(3):
        m = v & np.isfinite(dt[:, i])
        axs[2].scatter(hp[m, i] * 100, dt[m, i] * 100, s=5, label="xyz"[i])
    axs[2].set_xlabel("odom high-pass (cm)"); axs[2].set_ylabel("recovered dt (cm)")
    axs[2].set_title("C: recovered vs odom jitter"); axs[2].legend(fontsize=8); axs[2].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(out, "rigid.png"), dpi=105); plt.close(fig)


if __name__ == "__main__":
    main()
