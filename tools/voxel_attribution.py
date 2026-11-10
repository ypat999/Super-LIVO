#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""高跳变体素的几何—颜色双假设归因诊断（纯离线，不改任何算法代码）。

输入：blackframe_voxel_replay.py 产出的 voxel_observations.csv.gz
      （只用 pool_eligible=1 的观测，即真正进入 median 池的样本）。

三组样本（每组目标 200~500，高跳变组另留最严重 50 个逐帧可视化）：
  - high      高跳变：体素内亮度 std 大 且 邻接体素颜色跳变 > JUMP_THR
  - low       低跳变对照：同场景、n_obs 分布匹配、颜色稳定
  - boundary  边界：深度边缘（邻格沿法向质心差大）或轮廓（空邻格多）

双假设检测：
  几何混入 —— 5x5x5 邻域拟合局部平面，观测点到局部表面的有符号距离：
      分层间隙（20-80% 切分最大 gap）、跨帧质心最大位移、跨帧法向一致性。
  颜色波动 —— 逐帧平均亮度序列 std；lum 与 帧号/有符号距离/视角 cos/相机距离 的相关。

分类不强制二选一：geometric_mixing / color_variation / mixed / undetermined，
并保留 stable_but_adjacent_jump 标志（XYZ、RGB 都稳但邻格跳变——指向采样/栅格化口径）。

输出：attribution/voxel_classification.csv
      attribution/diagnostic_summary.json
      attribution/high_jump_voxels/<voxel_id>.png （最严重 50 个）
"""
import argparse
import gzip
import json
import os
import sys

import numpy as np
from scipy.stats import spearmanr, fisher_exact
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

VOX = 0.01
JUMP_THR = 40.0
BLACK_LUM = 3.0
BAG = "/home/ywj/slam_data/singleframe/sl_frames"
OBS = "/home/ywj/slam_data/singleframe/blackframe_replay/voxel_observations.csv.gz"
SUM = "/home/ywj/slam_data/singleframe/blackframe_replay/summary.json"
OUT = "/home/ywj/slam_data/singleframe/blackframe_replay/attribution"

# ---- 判定阈值（全部显式，避免"只看 XYZ 方差"陷阱） --------------------------
# 数据现实：450k 体素中 94% 只有 1 次观测，跨帧融合只发生在 27,397 个多观测体素；
# 修正投影后栅格椒盐 99.7% 落在多观测体素上。因此分组围绕"体素内跨帧分歧"定义：
RANGE_HI = 40.0                     # 高跳变组：体素内观测亮度极差 >=40（7,565 个）
RANGE_LO = 10.0                     # 低跳变对照：极差 <=10 且无邻格跳变（~12k 个）
JUMP_LO = 10.0
GAP_STRATA = 0.015                  # 分层：局部表面有符号距离的分裂间隙 >=1.5cm
SPREAD_GEO = 0.02                   # 几何混入证据：跨帧质心位移 >=2cm
GAP_STRATA_GEO = 0.02               # 或分层两组均值间距 >=2cm
STD_COL = 25.0                      # 颜色波动证据：逐帧平均亮度 std >=25
COL_STRATA = 25.0                   # 几何混入成立还需两组颜色差 >=25
DEPTH_EDGE = 0.03                   # 边界组：邻格沿法向质心差 >=3cm
N_MIN = 2                           # 归因分析最少观测数（>=2 才有跨帧证据）


# ------------------------------------------------------------------ 数据读取
def load_obs(path):
    vid, fr, P, RGB, EL = [], [], [], [], []
    with gzip.open(path, "rt") as f:
        f.readline()
        for line in f:
            p = line.replace(",", " ").split()
            vid.append(int(p[0]))
            fr.append(int(p[1]))
            P.append((float(p[3]), float(p[4]), float(p[5])))
            RGB.append((int(p[6]), int(p[7]), int(p[8])))
            EL.append(p[10].strip() == "1")
    return (np.array(vid, np.int64), np.array(fr, np.int32),
            np.array(P, np.float64), np.array(RGB, np.float64), np.array(EL, bool))


def unpack_vox(key):
    a0 = ((key >> 32) & 0xFFFF) - 32768
    a1 = ((key >> 16) & 0xFFFF) - 32768
    a2 = (key & 0xFFFF) - 32768
    return np.stack([a0, a1, a2], 1).astype(np.int32)


def pack16(v):
    a = v.astype(np.int64) + 32768
    return (a[:, 0] << 32) | (a[:, 1] << 16) | a[:, 2]


def load_odom(uri):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    odom = []
    while r.has_next():
        topic, payload, _ = r.read_next()
        if "odom" not in topic or not isinstance(payload, (bytes, bytearray)):
            continue
        m = deserialize_message(payload, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        odom.append((t, np.array([p.x, p.y, p.z])))
    odom.sort(key=lambda s: s[0])
    return odom


def make_cam_interp(odom):
    from scipy.spatial.transform import Slerp, Rotation
    st = np.array([o[0] for o in odom])
    pos = np.array([o[1] for o in odom])

    def cam_at(t):
        if t <= st[0]:
            return pos[0]
        if t >= st[-1]:
            return pos[-1]
        j = int(np.searchsorted(st, t))
        if st[j] == t:
            return pos[j]
        a = (t - st[j - 1]) / (st[j] - st[j - 1])
        return (1 - a) * pos[j - 1] + a * pos[j]

    return cam_at


def corr(a, b):
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 5:
        return float("nan")
    a, b = a[m], b[m]
    if a.std() < 1e-9 or b.std() < 1e-9:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


# ------------------------------------------------------------------ 全局体素统计
def voxel_table(vid_el, fr_el, P_el, RGB_el):
    uniq, inv = np.unique(vid_el, return_inverse=True)
    m = len(uniq)
    n = np.bincount(inv, minlength=m)
    L = 0.299 * RGB_el[:, 0] + 0.587 * RGB_el[:, 1] + 0.114 * RGB_el[:, 2]
    lmean = np.bincount(inv, weights=L, minlength=m) / n
    lstd = np.sqrt(np.maximum(np.bincount(inv, weights=L * L, minlength=m) / n - lmean ** 2, 0))
    chan_std = []
    for c in range(3):
        x = RGB_el[:, c]
        mu = np.bincount(inv, weights=x, minlength=m) / n
        sd = np.sqrt(np.maximum(np.bincount(inv, weights=x * x, minlength=m) / n - mu ** 2, 0))
        chan_std.append(sd)
    max_chan_std = np.maximum(np.maximum(chan_std[0], chan_std[1]), chan_std[2])
    cent = np.stack([np.bincount(inv, weights=P_el[:, c], minlength=m) / n for c in range(3)], 1)
    u2, i2 = np.unique(inv.astype(np.int64) * 64 + fr_el, return_inverse=True)
    nframes = np.bincount(i2, minlength=m)
    lmax = np.zeros(m); np.maximum.at(lmax, inv, L)
    lmin = np.full(m, np.inf); np.minimum.at(lmin, inv, L)
    return dict(uniq=uniq, inv=inv, n=n, L=L, lmean=lmean, lstd=lstd,
                max_chan_std=max_chan_std, cent=cent, nframes=nframes,
                lum_range=lmax - lmin)


def adjacency(tb):
    """6 邻接：最大亮度跳变、空邻格数、最大质心位移。"""
    uniq, lmean, cent = tb["uniq"], tb["lmean"], tb["cent"]
    vox_u = unpack_vox(uniq)
    m = len(uniq)
    NB = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    adj = np.full(m, np.nan)
    empty = np.zeros(m, np.int64)
    cshift = np.full(m, np.nan)
    for d in NB:
        nk = pack16(vox_u + np.array(d))
        j = np.searchsorted(uniq, nk)
        jc = np.clip(j, 0, m - 1)
        hit = uniq[jc] == nk
        tmp = np.full(m, np.nan)
        tmp[hit] = np.abs(lmean[jc[hit]] - lmean[hit])
        adj = np.fmax(np.nan_to_num(adj, nan=-np.inf), np.where(hit, tmp, -np.inf))
        adj = np.where(np.isneginf(adj), np.nan, adj)
        tmpc = np.full(m, np.nan)
        tmpc[hit] = np.linalg.norm(cent[jc[hit]] - cent[hit], axis=1)
        cshift = np.fmax(np.nan_to_num(cshift, nan=-np.inf), np.where(hit, tmpc, -np.inf))
        cshift = np.where(np.isneginf(cshift), np.nan, cshift)
        empty += ~hit
    adj = np.nan_to_num(adj, nan=0.0)
    cshift = np.nan_to_num(cshift, nan=0.0)
    return adj, empty, cshift


# ------------------------------------------------------------------ 逐体素归因
def analyze(g, tb, order, starts, P_el, RGB_el, fr_el, cam, vox_u, adj, empty):
    uniq, n, lmean, cent = tb["uniq"], tb["n"], tb["lmean"], tb["cent"]
    r0, r1 = starts[g], starts[g] + n[g]
    ridx = order[r0:r1]
    P, RGB, F, L = P_el[ridx], RGB_el[ridx], fr_el[ridx], tb["L"][ridx]
    nn = len(P)
    vox_c = vox_u[g]

    # ---- 5x5x5 邻域点拟合局部平面 ----
    offs = [(a, b, c) for a in range(-2, 3) for b in range(-2, 3) for c in range(-2, 3)]
    rows = []
    for o in offs:
        nk = pack16((vox_c + np.array(o))[None, :])[0]
        j = np.searchsorted(uniq, nk)
        if j < len(uniq) and uniq[j] == nk:
            rows.append((starts[j], starts[j] + n[j]))
    if rows:
        idx2 = np.concatenate([np.arange(a, b) for a, b in rows])
        Q = P_el[idx2]
    else:
        Q = P
    plane_ok = len(Q) >= 12
    c = cent[g]
    nrm = np.array([0.0, 0.0, 1.0])
    u_ax = np.array([1.0, 0.0, 0.0])
    if plane_ok:
        for _ in range(2):
            cc = Q.mean(0)
            Qc = Q - cc
            w, V = np.linalg.eigh(Qc.T @ Qc / len(Qc))
            nrm, u_ax = V[:, 0], V[:, 1]
            d = (Q - cc) @ nrm
            inl = np.abs(d) < 0.02
            if inl.sum() >= 12:
                Q = Q[inl]
            else:
                c = cc
                break
        c = Q.mean(0)
    Cmean = cam[F].mean(0)
    if plane_ok and (Cmean - c) @ nrm < 0:
        nrm = -nrm
    d_signed = (P - c) @ nrm if plane_ok else np.full(nn, np.nan)

    # ---- 分层（有符号距离 20-80% 切分最大间隙；n=2 直接两点间距）----
    stratified, gap, strata_color_gap = False, np.nan, np.nan
    if plane_ok and nn >= 2:
        o = np.argsort(d_signed)
        ds, Ls = d_signed[o], L[o]
        if nn == 2:
            gap = float(ds[1] - ds[0])
            k = 1
        else:
            lo, hi = int(np.ceil(0.2 * nn)), int(np.floor(0.8 * nn))
            if hi > lo:
                gaps = ds[lo:hi] - ds[lo - 1:hi - 1]
                k = int(np.argmax(gaps)) + lo
                gap = float(ds[k] - ds[k - 1])
        if not np.isnan(gap) and gap >= GAP_STRATA:
            stratified = True
            strata_color_gap = abs(float(Ls[:k].mean() - Ls[k:].mean()))

    # ---- 跨帧质心位移 ----
    frs = np.unique(F)
    cents = np.array([P[F == f].mean(0) for f in frs])
    spread = 0.0
    if len(cents) >= 2:
        dd = cents[:, None, :] - cents[None, :, :]
        spread = float(np.linalg.norm(dd, axis=2).max())
    # 逐帧质心沿法向偏移
    f_off = cents @ nrm - c @ nrm if plane_ok else np.full(len(frs), np.nan)

    # ---- 逐帧颜色 ----
    f_lum = np.array([L[F == f].mean() for f in frs])
    pf_std = float(f_lum.std(ddof=0)) if len(frs) >= 3 else np.nan

    # ---- 视角/距离相关性 ----
    Cv = cam[F]
    vec = Cv - P
    dist = np.linalg.norm(vec, axis=1)
    cosv = (vec * nrm).sum(1) / np.maximum(dist, 1e-9) if plane_ok else np.full(nn, np.nan)

    # ---- 边界标志 ----
    silhouette = int(empty[g] >= 2)
    depth_edge = 0
    if plane_ok:
        NB = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
        for d in NB:
            nk = pack16((vox_c + np.array(d))[None, :])[0]
            j = np.searchsorted(uniq, nk)
            if j < len(uniq) and uniq[j] == nk:
                if abs((cent[j] - cent[g]) @ nrm) >= DEPTH_EDGE:
                    depth_edge = 1
                    break

    geo_strong = bool(spread >= SPREAD_GEO or
                      (stratified and not np.isnan(gap) and gap >= GAP_STRATA_GEO))
    col_strong = bool((not np.isnan(pf_std) and pf_std >= STD_COL) or
                      (tb["lum_range"][g] >= RANGE_HI))
    if nn < N_MIN:
        cls = "insufficient"
    elif len(frs) < 2:
        cls = "single_frame_multi"
    elif geo_strong and stratified and strata_color_gap >= COL_STRATA:
        cls = "geometric_mixing"
    elif geo_strong and col_strong:
        cls = "mixed"
    elif (not geo_strong) and col_strong:
        cls = "color_variation"
    else:
        cls = "undetermined"
    stable_jump = bool((not geo_strong) and (not col_strong) and adj[g] >= JUMP_THR)

    # 两观测体素（占多数）：体素内相关不可算，改为跨体素池化的逐帧差分
    pair = None
    if nn == 2 and plane_ok and len(frs) == 2:
        o2 = np.argsort(F)
        pair = (float(L[o2][1] - L[o2][0]), float(d_signed[o2][1] - d_signed[o2][0]),
                float(cosv[o2][1] - cosv[o2][0]), float(np.log(dist[o2][1] / dist[o2][0])))

    return dict(g=g, nn=nn, nframes=len(frs), lmean=float(lmean[g]),
                intra_lum_std=float(tb["lstd"][g]), max_chan_std=float(tb["max_chan_std"][g]),
                lum_range=float(tb["lum_range"][g]),
                adj_jump=float(adj[g]), frame_centroid_spread_m=round(spread, 4),
                stratified=int(stratified), strata_gap_m=round(float(gap), 4) if not np.isnan(gap) else np.nan,
                strata_color_gap=round(float(strata_color_gap), 1) if not np.isnan(strata_color_gap) else np.nan,
                perframe_lum_std=round(float(pf_std), 1) if not np.isnan(pf_std) else np.nan,
                corr_lum_frame=corr(L, F.astype(float)),
                corr_lum_dsigned=corr(L, d_signed),
                corr_lum_cosview=corr(L, cosv),
                corr_lum_dist=corr(L, dist),
                plane_ok=int(plane_ok), geo_strong=int(geo_strong), col_strong=int(col_strong),
                _pair=pair,
                stable_but_adjacent_jump=int(stable_jump),
                depth_edge=depth_edge, silhouette=silhouette, cls=cls,
                _P=P, _RGB=RGB, _F=F, _L=L, _frs=frs, _f_lum=f_lum, _f_off=f_off,
                _c=c, _nrm=nrm, _u=u_ax, _d=d_signed, _vox=vox_c)


# ------------------------------------------------------------------ 可视化
def plot_voxel(res, path):
    P, F, L, d = res["_P"], res["_F"], res["_L"], res["_d"]
    fig, axs = plt.subplots(2, 2, figsize=(11, 8.5))
    ax = axs[0, 0]
    sc = ax.scatter(P[:, 0], P[:, 1], c=F, cmap="tab20", s=18)
    vc = res["_vox"]
    for i in (-1, 0, 1):
        for j in (-1, 0, 1):
            ax.add_patch(Rectangle((vc[0] * VOX + i * VOX, vc[1] * VOX + j * VOX),
                                   VOX, VOX, fill=False,
                                   lw=1.5 if (i == 0 and j == 0) else 0.4,
                                   ec="red" if (i == 0 and j == 0) else "gray"))
    ax.set_title("top-down x-y, color=frame_id")
    plt.colorbar(sc, ax=ax, label="frame")
    ax = axs[0, 1]
    if res["plane_ok"]:
        u = (P - res["_c"]) @ res["_u"]
        ax.scatter(u * 100, d * 100, c=L, cmap="viridis", s=18)
        plt.colorbar(ax.collections[0], ax=ax, label="lum")
        ax.axvspan(-0.5, 0.5, color="red", alpha=0.12)
        ax.set_xlabel("in-plane u (cm)")
        ax.set_ylabel("signed dist to local surface (cm)")
        ax.set_title(f"plane coords  stratified={res['stratified']} gap={res['strata_gap_m']}")
    else:
        ax.text(0.5, 0.5, "plane fit failed", ha="center")
    ax = axs[1, 0]
    ax.bar(res["_frs"], res["_f_lum"], width=0.8)
    ax.axhline(res["lmean"], color="gray", lw=0.8)
    ax.set_xlabel("frame")
    ax.set_ylabel("mean lum per frame")
    ax.set_title(f"per-frame color  std={res['perframe_lum_std']}")
    ax = axs[1, 1]
    ax.scatter(res["_frs"], res["_f_off"] * 100, c=res["_f_lum"], cmap="viridis", s=40)
    ax.axhline(0, color="gray", lw=0.8)
    ax.set_xlabel("frame")
    ax.set_ylabel("frame centroid offset along normal (cm)")
    ax.set_title(f"geo spread={res['frame_centroid_spread_m']*100:.1f}cm adj={res['adj_jump']:.0f}")
    fig.suptitle(f"voxel {res.get('voxel_id', res['g'])}  class={res['cls']}  "
                 f"n={res['nn']} frames={res['nframes']} lum_range={res['lum_range']:.0f}", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


# ------------------------------------------------------------------ 主流程
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-high", type=int, default=500)
    ap.add_argument("--n-low", type=int, default=500)
    ap.add_argument("--n-bnd", type=int, default=300)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    fig_dir = os.path.join(OUT, "high_jump_voxels")
    os.makedirs(fig_dir, exist_ok=True)
    for fn in os.listdir(fig_dir):  # 清理旧图，避免不同轮次混叠
        if fn.endswith(".png"):
            os.remove(os.path.join(fig_dir, fn))

    vid, fr, P, RGB, EL = load_obs(OBS)
    vid_el, fr_el, P_el, RGB_el = vid[EL], fr[EL], P[EL], RGB[EL]
    print(f"观测 {len(vid):,} 行，入池 {EL.sum():,} 行")
    tb = voxel_table(vid_el, fr_el, P_el, RGB_el)
    uniq, n = tb["uniq"], tb["n"]
    adj, empty, cshift = adjacency(tb)
    vox_u = unpack_vox(uniq)
    m = len(uniq)
    print(f"体素 {m:,}；邻跳变>{JUMP_THR:.0f} 的体素 {int((adj > JUMP_THR).sum()):,}")

    order = np.argsort(tb["inv"], kind="stable")
    starts = np.zeros(m, np.int64)
    np.cumsum(n[:-1], out=starts[1:])

    rng = np.random.default_rng(7)
    ok_n = n >= N_MIN
    hi_idx = np.where(ok_n & (tb["lum_range"] >= RANGE_HI))[0]
    hi_sel = rng.choice(hi_idx, size=min(a.n_high, len(hi_idx)), replace=False)
    hi_set = set(hi_sel.tolist())

    lo_pool = np.where(ok_n & (tb["lum_range"] <= RANGE_LO) & (adj <= JUMP_LO))[0]
    bins = [2, 3, 4, 6, 10, 10 ** 9]
    lo_sel = []
    for i in range(len(bins) - 1):
        hb = hi_sel[(n[hi_sel] >= bins[i]) & (n[hi_sel] < bins[i + 1])]
        lb = lo_pool[(n[lo_pool] >= bins[i]) & (n[lo_pool] < bins[i + 1])]
        k = min(len(hb), len(lb), max(20, a.n_low // (len(bins) - 1)))
        if k > 0:
            lo_sel.extend(rng.choice(lb, size=k, replace=False).tolist())
    lo_sel = np.array(lo_sel[:a.n_low])
    lo_set = set(lo_sel.tolist())

    bnd_pool = np.where(ok_n & ((empty >= 2) | (cshift >= DEPTH_EDGE)))[0]
    bnd_pool = np.array([x for x in bnd_pool if x not in hi_set and x not in lo_set])
    bnd_sel = rng.choice(bnd_pool, size=min(a.n_bnd, len(bnd_pool)), replace=False)

    groups = {}
    for tag, idxs in (("high", hi_sel), ("low", lo_sel), ("boundary", bnd_sel)):
        for g in idxs:
            groups[int(g)] = tag
    print(f"样本：high={len(hi_sel)} low={len(lo_sel)} boundary={len(bnd_sel)}")

    # ---- 相机中心 ----
    summ = json.load(open(SUM))
    odom = load_odom(BAG)
    cam_at = make_cam_interp(odom)
    cam = np.array([cam_at(r["t_cloud"]) for r in summ["per_frame"]])
    print(f"odom {len(odom)} 条，相机中心 {cam.shape}")

    worst = sorted(hi_sel.tolist(),
                   key=lambda g: -(tb["lum_range"][g] * max(adj[g], 1.0)))[:50]
    worst = set(worst)

    res_list = []
    for i, g in enumerate(sorted(groups)):
        res = analyze(g, tb, order, starts, P_el, RGB_el, fr_el, cam, vox_u, adj, empty)
        res["group"] = groups[g]
        res["voxel_id"] = int(uniq[g])
        res_list.append(res)
        if g in worst:
            plot_voxel(res, os.path.join(OUT, "high_jump_voxels", f"{uniq[g]}.png"))
        if (i + 1) % 300 == 0:
            print(f"  归因 {i+1}/{len(groups)}")

    # ---- 统计 ----
    def col(name):
        return np.array([r[name] for r in res_list], float)

    grp = np.array([r["group"] for r in res_list])
    geo = col("geo_strong")
    cls = [r["cls"] for r in res_list]
    stat = {}
    for tag in ("high", "low", "boundary"):
        gm = grp == tag
        cnt = {c: int(sum(1 for r, k in zip(res_list, gm) if k and r["cls"] == c))
               for c in ("geometric_mixing", "color_variation", "mixed",
                         "undetermined", "insufficient", "single_frame_multi")}
        stat[tag] = dict(
            n=int(gm.sum()),
            geo_strong_rate=round(float(geo[gm].mean()), 4),
            stratified_rate=round(float(col("stratified")[gm].mean()), 4),
            mean_frame_centroid_spread_cm=round(float(col("frame_centroid_spread_m")[gm].mean() * 100), 2),
            mean_strata_gap_cm=round(float(np.nanmean(col("strata_gap_m")[gm])) * 100, 2),
            mean_perframe_lum_std=round(float(np.nanmean(col("perframe_lum_std")[gm])), 1),
            mean_intra_lum_std=round(float(col("intra_lum_std")[gm].mean()), 1),
            mean_adj_jump=round(float(col("adj_jump")[gm].mean()), 1),
            stable_but_adjacent_jump=int(sum(1 for r, k in zip(res_list, gm) if k and r["stable_but_adjacent_jump"])),
            corr_lum_dsigned_median=round(float(np.nanmedian(col("corr_lum_dsigned")[gm])), 3),
            corr_lum_cosview_median=round(float(np.nanmedian(col("corr_lum_cosview")[gm])), 3),
            corr_lum_frame_median=round(float(np.nanmedian(col("corr_lum_frame")[gm])), 3),
            classes=cnt)
    gh = int(geo[grp == "high"].sum()); nh = int((grp == "high").sum())
    gl = int(geo[grp == "low"].sum()); nl = int((grp == "low").sum())
    try:
        odds, p = fisher_exact([[gh, nh - gh], [gl, nl - gl]])
    except Exception:
        odds, p = float("nan"), float("nan")
    strat_all = col("stratified")
    try:
        odds2, p2 = fisher_exact([[int(strat_all[grp == "high"].sum()), nh - int(strat_all[grp == "high"].sum())],
                                  [int(strat_all[grp == "low"].sum()), nl - int(strat_all[grp == "low"].sum())]])
    except Exception:
        odds2, p2 = float("nan"), float("nan")
    rho1 = spearmanr(col("adj_jump"), col("frame_centroid_spread_m")).statistic
    rho2 = spearmanr(col("adj_jump"), col("perframe_lum_std")).statistic
    rho3 = spearmanr(col("strata_gap_m"), col("strata_color_gap")).statistic

    # 池化逐帧差分（两观测体素，样本量大）：Δlum 与 视角/表面距离/距离 差的相关
    pooled = {}
    for tag in ("high", "low", "boundary"):
        arr = np.array([r["_pair"] for r in res_list
                        if r["group"] == tag and r["_pair"] is not None])
        if len(arr) >= 20:
            dL, dd, dcos, dld = arr.T
            pooled[tag] = dict(
                n=len(arr),
                mean_abs_dLum=round(float(np.abs(dL).mean()), 1),
                corr_dLum_dcosview=round(corr(dL, dcos), 3),
                corr_dLum_ddist_to_surface=round(corr(dL, dd), 3),
                corr_dLum_dlogdist=round(corr(dL, dld), 3),
                median_abs_dcosview=round(float(np.median(np.abs(dcos))), 3))
        else:
            pooled[tag] = dict(n=int(len(arr)))
    summary = dict(
        thresholds=dict(RANGE_HI=RANGE_HI, RANGE_LO=RANGE_LO, JUMP_LO=JUMP_LO,
                        GAP_STRATA_cm=GAP_STRATA * 100,
                        SPREAD_GEO_cm=SPREAD_GEO * 100, STD_COL=STD_COL, COL_STRATA=COL_STRATA,
                        DEPTH_EDGE_cm=DEPTH_EDGE * 100, N_MIN=N_MIN),
        group_stats=stat,
        high_vs_low_geo_strong=dict(high=f"{gh}/{nh}", low=f"{gl}/{nl}",
                                    fisher_or=round(float(odds), 2), fisher_p=float(f"{p:.3g}")),
        high_vs_low_stratified=dict(high_or=round(float(odds2), 2), fisher_p2=float(f"{p2:.3g}")),
        spearman=dict(adj_jump_vs_centroid_spread=round(float(rho1), 3),
                      adj_jump_vs_perframe_std=round(float(rho2), 3),
                      strata_gap_vs_strata_color_gap=round(float(rho3), 3)),
        pooled_pair_diff=pooled,
        worst_visualized=len(worst))

    with open(os.path.join(OUT, "diagnostic_summary.json"), "w") as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)

    cols = ["voxel_id", "group", "nn", "nframes", "lmean", "intra_lum_std", "max_chan_std",
            "lum_range", "adj_jump", "frame_centroid_spread_m", "stratified", "strata_gap_m",
            "strata_color_gap", "perframe_lum_std", "corr_lum_frame", "corr_lum_dsigned",
            "corr_lum_cosview", "corr_lum_dist", "plane_ok", "geo_strong", "col_strong",
            "stable_but_adjacent_jump", "depth_edge", "silhouette", "cls"]
    with open(os.path.join(OUT, "voxel_classification.csv"), "w") as f:
        f.write(",".join(cols) + "\n")
        for r in res_list:
            f.write(",".join("" if r[c] is None else (f"{r[c]:.4f}" if isinstance(r[c], float) else str(r[c]))
                             for c in cols) + "\n")

    print("\n=== diagnostic_summary ===")
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    print(f"\n产物：{OUT}/voxel_classification.csv, diagnostic_summary.json, high_jump_voxels/*.png")


if __name__ == "__main__":
    main()
