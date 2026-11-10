#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""指定 ROI 的离散度归因（无需法向/平面性假设，适用于凸起字）。

对每个球形 ROI 计算三组量：
  1) 轴向厚度：pooled 的 P95−P5 与"各帧单独算后取中位"的差（x/y/z 三轴 + 沿视线）。
     —— 非平面区域也能算，因为它不依赖局部法向。
  2) 同帧最近邻 vs 跨帧最近邻：
        d_same  = 点到"同帧另一点"的最近距离   → 单帧自身密度/footprint
        d_cross = 点到"异帧点"的最近距离       → 单帧密度 + 跨帧错位
     若 d_cross ≈ d_same，重影不显著；若 d_cross >> d_same，重影是主因。
  3) 每帧三维偏移：对每个点取 k 近邻中"异帧邻居"的均值向量 mean(q_j − q_i)，
     按帧聚合 = 该帧整体偏离其余帧共识面的 3D 向量（LOO 近似，不重建树）。
     这是最贴近肉眼"这帧的字母偏了多远"的量。

用法：python3 tools/roi_scatter.py [--roi "x,y,z,r,name" ...]（可多次）
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame_wall_offset import load

DEF_ROI = ["-11.09,-24.31,1.99,0.8,BK_text", "-13.65,-26.99,4.96,2.0,flat_wall"]


def thickness(P, fr, nf, axes=range(3)):
    pooled = {}
    per_frame = {}
    for ax in axes:
        v = P[:, ax]
        pooled[ax] = np.percentile(v, 95) - np.percentile(v, 5)
        s = []
        for f in np.unique(fr):
            q = v[fr == f]
            if len(q) >= 30:
                s.append(np.percentile(q, 95) - np.percentile(q, 5))
        per_frame[ax] = float(np.median(s)) if len(s) >= 5 else np.nan
    return pooled, per_frame


def nn_metrics(P, fr, k=32):
    tree = cKDTree(P)
    dd, ii = tree.query(P, k=min(k, len(P)))
    same, cross, off = [], [], np.zeros((len(P), 3))
    for i in range(len(P)):
        nb = ii[i]
        valid = nb != i
        fv = fr[nb[valid]]
        dv = dd[i][valid]
        d_same = dv[fv == fr[i]]
        d_cross = dv[fv != fr[i]]
        same.append(d_same.min() if len(d_same) else np.nan)
        cross.append(d_cross.min() if len(d_cross) else np.nan)
        if len(d_cross):
            sel = nb[valid][fv != fr[i]]
            off[i] = (P[sel] - P[i]).mean(0)
    return np.array(same), np.array(cross), off


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="/home/ywj/slam_data/pcd/retail_framecolor/PCD")
    ap.add_argument("--out", default="/home/ywj/slam_data/pcd/retail_framecolor/spread")
    ap.add_argument("--roi", action="append", default=DEF_ROI)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    P, fr, pose, names = load(a.src)
    nf = len(names)
    Pd = P.astype(np.float64)
    out = {}
    for spec in a.roi:
        x, y, z, r, *lab = spec.split(",")
        c = np.array([float(x), float(y), float(z)])
        r = float(r)
        name = lab[0] if lab else f"roi_{x}_{y}"
        m = np.linalg.norm(Pd - c, axis=1) <= r
        Q, F = Pd[m], fr[m]
        uf, cnt = np.unique(F, return_counts=True)
        print(f"\n=== {name}  中心{c.round(2).tolist()} r={r}m ===")
        print(f"点数={m.sum()}  帧数={len(uf)}  每帧点数中位={int(np.median(cnt))} "
              f"(min {cnt.min()}, max {cnt.max()})")
        if m.sum() < 500 or len(uf) < 10:
            print("  样本不足，跳过")
            continue

        pooled, perfr = thickness(Q, F, nf)
        res = {"n_points": int(m.sum()), "n_frames": int(len(uf)),
               "pts_per_frame_med": int(np.median(cnt)), "axis_thickness_cm": {}}
        for ax, nm in enumerate("xyz"):
            p_, s_ = pooled[ax] * 100, perfr[ax] * 100
            res["axis_thickness_cm"][nm] = dict(pooled=round(p_, 2), single=round(s_, 2),
                                                extra=round(max(p_ - s_, 0), 2))
            print(f"  {nm} 轴厚度：多帧={p_:5.2f}cm  单帧={s_:5.2f}cm  净增={max(p_-s_,0):5.2f}cm")

        ds, dc, off = nn_metrics(Q, F)
        ok = np.isfinite(ds) & np.isfinite(dc)
        r_cm = np.median(dc[ok] * 100) / max(np.median(ds[ok] * 100), 1e-9)
        print(f"  最近邻：同帧中位={np.median(ds[ok])*100:.2f}cm  跨帧中位={np.median(dc[ok])*100:.2f}cm"
              f"  比值={r_cm:.2f}")
        print(f"  跨帧最近邻 P90={np.percentile(dc[ok],90)*100:.2f}cm  "
              f"同帧最近邻 P90={np.percentile(ds[ok],90)*100:.2f}cm")
        res["nn_same_cm"] = round(float(np.median(ds[ok]) * 100), 2)
        res["nn_cross_cm"] = round(float(np.median(dc[ok]) * 100), 2)
        res["nn_ratio"] = round(float(r_cm), 3)
        res["nn_cross_p90_cm"] = round(float(np.percentile(dc[ok], 90) * 100), 2)

        mag = np.linalg.norm(off, axis=1) * 100
        per_f = []
        for f in uf:
            sel = (F == f) & np.isfinite(mag)
            if sel.sum() >= 30:
                per_f.append(np.median(mag[sel]))
        per_f = np.array(per_f)
        print(f"  每帧整体偏移（相对其余帧共识面）：中位={np.median(per_f):.2f}cm  "
              f"P75={np.percentile(per_f,75):.2f}  P90={np.percentile(per_f,90):.2f}  "
              f"最大={per_f.max():.2f}cm  帧数={len(per_f)}")
        res["frame_offset_cm"] = dict(med=round(float(np.median(per_f)), 2),
                                      p75=round(float(np.percentile(per_f, 75)), 2),
                                      p90=round(float(np.percentile(per_f, 90)), 2),
                                      mx=round(float(per_f.max()), 2), n=int(len(per_f)))
        # 时间形态：偏移是否集中在少数帧
        res["share_frames_gt_2x_med"] = round(float(np.mean(per_f > 2 * np.median(per_f))), 3)
        print(f"  偏移 > 2×中位 的帧占比={res['share_frames_gt_2x_med']:.3f}")
        out[name] = res

    with open(os.path.join(a.out, "roi_scatter.json"), "w") as f:
        json.dump(out, f, indent=1, ensure_ascii=False)
    print(f"\n结果：{a.out}/roi_scatter.json")


if __name__ == "__main__":
    main()
