#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单一平面沿法向的多帧厚度/分层（半径扫描，排除球内混入的其他表面）。

与 visual_scatter.py 的区别：那里用 0.5m 体素的"局部法向"，对整场景统计；这里针对
用户指定的某个面，先在 ROI 内 RANSAC 拟合出该面的平面，**只保留该面内点**
（|d|<tol），再沿该面法向 n 统计深度 d：
  thick_multi  = P95(d)-P5(d)  全部帧合并          ← 肉眼看到的该面糊团厚度
  thick_single = 各帧单独算后取中位                 ← 单帧 footprint/表面起伏
  layer_std    = 各帧平均深度 d̄_f 的加权 std        ← 千层饼层间距（纯重影）
  layer_range  = max(d̄_f)-min(d̄_f)
并做半径扫描，看指标随 ROI 缩小是否稳定（若缩小后 layer_std 不降，说明重影是真的；
若明显下降，说明之前混入了其他表面）。
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame_wall_offset import load

ROIS = [(-11.09, -24.31, 1.99, "BK_text"), (-13.65, -26.99, 4.96, "flat_wall")]
RADII = [float(v) for v in os.environ.get("RADII", "0.4,0.8,1.5").split(",")]


def ransac_plane(Q, seed=0, iters=600, tol=0.03):
    rng = np.random.default_rng(seed)
    S = Q[rng.choice(len(Q), size=min(50000, len(Q)), replace=False)]
    best = (0, None, None)
    for _ in range(iters):
        p = S[rng.choice(len(S), 3, replace=False)]
        c = p.mean(0)
        u, v = p[1] - p[0], p[2] - p[0]
        n = np.cross(u, v)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        cnt = int((np.abs((S - c) @ n) < tol).sum())
        if cnt > best[0]:
            best = (cnt, c, n)
    cnt, c, n = best
    for _ in range(4):
        sel = np.abs((Q - c) @ n) < tol
        c = Q[sel].mean(0)
        u, s, vt = np.linalg.svd(Q[sel] - c, full_matrices=False)
        n = vt[2]
    return c, n, float(sel.mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="/home/ywj/slam_data/pcd/retail_framecolor/PCD")
    ap.add_argument("--out", default="/home/ywj/slam_data/pcd/retail_framecolor/spread")
    ap.add_argument("--tol", type=float, default=0.08, help="该面内点的法向窗口")
    ap.add_argument("--min-per-frame", type=int, default=100)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    P, fr, pose, names = load(a.src)
    out = {}
    for x, y, z, name in ROIS:
        c0 = np.array([x, y, z])
        out[name] = {}
        print(f"\n=== {name} @ {c0.round(2).tolist()} ===")
        for r in RADII:
            # 逐分量 float32 求平方距离，避免整块 float64 副本（27.7M×3×8B）撑爆内存
            dx = P[:, 0] - np.float32(x)
            dy = P[:, 1] - np.float32(y)
            dz = P[:, 2] - np.float32(z)
            m = (dx * dx + dy * dy + dz * dz) <= np.float32(r * r)
            del dx, dy, dz
            if m.sum() < 2000:
                print(f"  r={r}m 点太少({m.sum()})")
                continue
            Q, F = P[m].astype(np.float64), fr[m]
            pc, n, frac = ransac_plane(Q)
            d = (Q - pc) @ n
            ins = np.abs(d) < a.tol
            Qi, Fi, di = Q[ins], F[ins], d[ins]
            if len(di) < 1000:
                print(f"  r={r}m 面内点不足({len(di)})")
                continue
            multi = (np.percentile(di, 95) - np.percentile(di, 5)) * 100
            per_th, means, wts = [], [], []
            for f in np.unique(Fi):
                s = di[Fi == f]
                if len(s) >= a.min_per_frame:
                    per_th.append((np.percentile(s, 95) - np.percentile(s, 5)) * 100)
                    means.append(s.mean())
                    wts.append(len(s))
            means, wts = np.array(means), np.array(wts, float)
            single = float(np.median(per_th))
            w = wts / wts.sum()
            mbar = float((w * means).sum())
            layer_std = float(np.sqrt((w * (means - mbar) ** 2).sum()) * 100)
            rng = np.percentile(means, 95) - np.percentile(means, 5)
            res = dict(n_inlier=int(len(di)), inlier_frac=round(float(frac), 3),
                       normal=[round(float(v), 3) for v in n],
                       n_frames=len(per_th), thick_multi_cm=round(float(multi), 2),
                       thick_single_cm=round(single, 2), ghost_extra_cm=round(multi - single, 2),
                       layer_std_cm=round(layer_std, 2), layer_p95p5_cm=round(float(rng) * 100, 2),
                       layer_max_min_cm=round((means.max() - means.min()) * 100, 2))
            out[name][f"r{r}"] = res
            print(f"  r={r}m 内点={len(di)}({frac*100:.0f}%) 帧={len(per_th)} | "
                  f"多帧厚度={multi:5.2f} 单帧={single:5.2f} 净增={multi-single:5.2f} | "
                  f"层间距std={layer_std:4.2f} P95-P5={rng*100:4.2f} 极差={res['layer_max_min_cm']:5.2f} cm")
    with open(os.path.join(a.out, "plane_normal_thickness.json"), "w") as f:
        json.dump(out, f, indent=1, ensure_ascii=False)
    print(f"\n结果：{a.out}/plane_normal_thickness.json")


if __name__ == "__main__":
    main()
