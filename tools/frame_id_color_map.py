#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按帧随机染色：区分"单帧矫正不足"与"跨帧位姿不统一（重影）"（纯离线）。

原理：
  voxel_observations.csv.gz 里的 pool_eligible 观测 = 最终地图几何（§12-13 已用
  0.1mm 成员判定验证），且自带帧索引。把每帧赋一个随机高饱和颜色导出 PCD：
    * 单色斑块本身就厚/糊           -> 单帧内矫正不足（去畸变/运动补偿问题）；
    * 单色斑块薄而锐利，不同颜色形成
      平行偏移的"千层饼"           -> 跨帧位姿不统一（重影）。
  另出一版顺序色相（色相=帧序），时间相邻帧颜色相邻，可看漂移是渐进还是跳变。

定量分解（ANOVA，体素级）：
  对 >=2 帧且 >=4 点的体素，沿邻域拟合平面法向取深度 d：
    跨帧方差 = "帧均值深度"的加权方差   -> 重影量
    帧内方差 = 点绕自身帧均值的池化方差 -> 单帧厚度
  报告两者中位数/分位数与方差占比（跨帧/(跨帧+帧内)）。

用法：
  python3 tools/frame_id_color_map.py [--out DIR] [--voxel 0.02] [--seed 7]
输出：
  frame_random.pcd   每帧随机色（主诊断图）
  frame_hue.pcd      色相=帧序（时间梯度图）
  frame_legend.json  帧号 -> 颜色/时刻/点数
  summary.json       ANOVA 分解统计
  preview_random.png 俯视 XY 散点预览
"""
import argparse
import colorsys
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from voxel_attribution import load_obs, OBS, pack16
from voxel_fusion_G import fit_planes


def write_pcd(path, xyz, rgb):
    n = len(xyz)
    pk = ((rgb[:, 0].astype(np.uint32) << 16) | (rgb[:, 1].astype(np.uint32) << 8)
          | rgb[:, 2].astype(np.uint32))
    body = np.zeros((n, 16), np.uint8)
    body[:, :12] = xyz.astype(np.float32).view(np.uint8).reshape(n, 12)
    body[:, 12:] = pk.view(np.uint8).reshape(n, 4)
    hdr = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
           "FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n"
           f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n")
    with open(path, "wb") as f:
        f.write(hdr.encode() + body.tobytes())


def frame_colors(nf, seed, mode):
    """每帧一个 RGB（uint8）。mode=random：随机色相；mode=hue：色相=帧序。"""
    rng = np.random.default_rng(seed)
    out = np.zeros((nf, 3), np.uint8)
    for i in range(nf):
        h = rng.uniform() if mode == "random" else (i / max(nf - 1, 1))
        r, g, b = colorsys.hsv_to_rgb(h, 0.95, 1.0)
        out[i] = (np.array([r, g, b]) * 255).astype(np.uint8)
    return out


def anova(P, fr, edge, max_vox=20000, seed=1):
    """体素级 跨帧(重影) vs 帧内(单帧厚度) 方差分解。返回统计 dict。"""
    vox = np.floor(P / edge).astype(np.int32)
    key = pack16(vox)
    uniq, inv = np.unique(key, return_inverse=True)
    m = len(uniq)
    n = np.bincount(inv, minlength=m).astype(np.float64)
    cent = np.stack([np.bincount(inv, weights=P[:, c], minlength=m) / n for c in range(3)], 1)

    nf = int(fr.max()) + 1
    key2 = inv.astype(np.int64) * (nf + 1) + fr.astype(np.int64)
    u2 = np.unique(key2)
    nvf = np.bincount(u2 // (nf + 1), minlength=m)          # 该体素被几帧观测

    cand = np.where((n >= 4) & (nvf >= 2))[0]
    if len(cand) == 0:
        return dict(n_voxels=0)
    rng = np.random.default_rng(seed)
    mfs = rng.choice(cand, size=min(len(cand), max_vox), replace=False) if len(cand) > max_vox else cand

    c0, nrm, ok = fit_planes(uniq, cent, n, mfs)

    order = np.argsort(inv, kind="stable")
    inv_s = inv[order]
    starts = np.searchsorted(inv_s, np.arange(m))

    between, within, gap, frac = [], [], [], []
    for gi, g in enumerate(mfs):
        if not ok[gi]:
            continue
        rid = order[starts[g]:starts[g] + int(n[g])]
        F = fr[rid]
        d = (P[rid] - c0[gi]) @ nrm[gi]
        uf, cnt = np.unique(F, return_counts=True)
        if len(uf) < 2:
            continue
        mf = np.stack([d[F == f].mean() for f in uf])
        w = cnt / cnt.sum()
        mbar = float((w * mf).sum())
        b_var = float((w * (mf - mbar) ** 2).sum())
        dev = d - mf[np.searchsorted(uf, F)]
        w_var = float((dev ** 2).sum() / max(len(d) - len(uf), 1))
        between.append(np.sqrt(b_var))
        within.append(np.sqrt(w_var))
        gap.append(float(mf.max() - mf.min()))
        tot = b_var + w_var
        frac.append(b_var / tot if tot > 1e-12 else np.nan)

    def q(a, mul=100.0):
        a = np.array(a) * mul if len(a) else np.zeros(1)
        return dict(med=float(np.median(a)), p75=float(np.percentile(a, 75)),
                    p90=float(np.percentile(a, 90)))

    return dict(n_voxels=int(len(between)),
                between_std_cm=q(between),          # 重影：帧间质心深度散布
                within_std_cm=q(within),            # 单帧厚度：帧内点散布
                ghost_range_cm=q(gap),              # 帧均值深度极差
                between_var_frac_med=float(np.nanmedian(frac)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--obs", default=OBS)
    ap.add_argument("--out", default="/home/ywj/slam_data/singleframe/frame_color")
    ap.add_argument("--voxel", type=float, default=0.02)
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    vid, fr, P, RGB, EL = load_obs(a.obs)
    fr, P = fr[EL].astype(np.int64), P[EL].astype(np.float64)
    nf = int(fr.max()) + 1
    print(f"地图观测 {len(P)} 点 / {nf} 帧")

    # ---- 两版染色 PCD + 图例 ----
    legend = {}
    for mode, fname in (("random", "frame_random.pcd"), ("hue", "frame_hue.pcd")):
        C = frame_colors(nf, a.seed, mode)
        rgb = C[fr]
        write_pcd(os.path.join(a.out, fname), P.astype(np.float32), rgb)
        if mode == "random":
            legend = {int(i): [int(C[i, 0]), int(C[i, 1]), int(C[i, 2]),
                               int((fr == i).sum())] for i in range(nf)}
            with open(os.path.join(a.out, "frame_legend.json"), "w") as f:
                json.dump(legend, f, indent=1)

    # ---- ANOVA：体素 2cm（主）+ 5cm（稳健性）----
    stats = {"n_points": int(len(P)), "n_frames": nf}
    for e in (a.voxel, 0.05):
        stats[f"anova_{int(e*100)}cm"] = anova(P, fr, e)
        s = stats[f"anova_{int(e*100)}cm"]
        if s.get("n_voxels"):
            print(f"[{int(e*100)}cm] 体素数={s['n_voxels']}  "
                  f"跨帧std中位={s['between_std_cm']['med']:.2f}cm "
                  f"帧内std中位={s['within_std_cm']['med']:.2f}cm "
                  f"重影极差中位={s['ghost_range_cm']['med']:.2f}cm "
                  f"跨帧方差占比中位={s['between_var_frac_med']:.2f}")

    with open(os.path.join(a.out, "summary.json"), "w") as f:
        json.dump(stats, f, indent=1)

    # ---- 俯视预览 PNG ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        rng = np.random.default_rng(0)
        idx = rng.choice(len(P), size=min(300000, len(P)), replace=False)
        C = frame_colors(nf, a.seed, "random")
        plt.figure(figsize=(10, 10))
        plt.scatter(P[idx, 0], P[idx, 1], s=0.3, c=C[fr[idx]] / 255.0, linewidths=0)
        plt.gca().set_aspect("equal")
        plt.title("per-frame random color (top view, XY)")
        plt.savefig(os.path.join(a.out, "preview_random.png"), dpi=150,
                    bbox_inches="tight")
        plt.close()
        print("preview_random.png 已输出")
    except Exception as e:
        print(f"PNG 预览跳过：{e}")


if __name__ == "__main__":
    main()
