#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跨帧三维散布（修正 frame_wall_offset 的度量盲区）。

之前报的 0.52cm 是"沿局部平面法向的深度散布"，并且体素筛选带平面性条件
（λmin/λmax≤0.02、厚度≤4cm、≥200 点）。这会系统性排除招牌凸起字这类
小面积、非平面、稀疏区域；而面向雷达的平面，位姿角误差主要产生**切向**位移
（15m 处 0.2° ≈ 5cm），法向位移只是二阶量。所以那个数字回答不了"肉眼看到的
离散半径"。

本脚本不做任何平面性/厚度筛选，逐体素计算每帧质心的三维散布并分解：
    between3D²  = Σ_f c_f|m_f − m̄|² / N          （= trace(帧间协方差)，肉眼看到的）
    betweenN²   = Σ_f c_f(d_f − d̄)² / N，d=n·m   （法向分层，对应旧指标）
    betweenT²   = between3D² − betweenN²          （切向涂抹，旧指标看不见）
其中 n 取该体素自身点云协方差的最小特征向量，m_f 是该帧在该体素内的质心。

输出：全场景分位数、>5cm 占比、最高散布体素位置，以及一张按 between3D 上色的
体素质心 PCD（可在 CloudCompare 里定位招牌）。

用法：python3 tools/spatial_spread.py [--edge 0.05] [--out DIR]
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame_wall_offset import load
from voxel_attribution import pack16
from frame_id_color_map import write_pcd

PAIRS = ((0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="/home/ywj/slam_data/pcd/retail_framecolor/PCD")
    ap.add_argument("--out", default="/home/ywj/slam_data/pcd/retail_framecolor/spread")
    ap.add_argument("--edge", type=float, default=0.05)
    ap.add_argument("--min-pts", type=int, default=50)
    ap.add_argument("--min-frames", type=int, default=5)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    P, fr, pose, names = load(a.src)
    nf = len(names)
    Pd = P.astype(np.float64)
    print(f"载入 {len(P)} 点 / {nf} 帧，体素 {a.edge*100:.0f}cm")

    uniq, inv = np.unique(pack16(np.floor(P / a.edge).astype(np.int32)), return_inverse=True)
    nvox = len(uniq)
    pk = inv.astype(np.int64) * nf + fr.astype(np.int64)
    upair, pinv = np.unique(pk, return_inverse=True)
    npair = len(upair)
    pv = upair // nf
    print(f"体素 {nvox}，(体素×帧) 对 {npair}")

    cnt = np.bincount(pinv, minlength=npair).astype(np.float64)
    s1 = np.stack([np.bincount(pinv, weights=Pd[:, c], minlength=npair) for c in range(3)], 1)
    s2 = np.stack([np.bincount(pinv, weights=Pd[:, i] * Pd[:, j], minlength=npair)
                   for i, j in PAIRS], 1)

    N = np.bincount(pv, weights=cnt, minlength=nvox)
    T1 = np.stack([np.bincount(pv, weights=s1[:, c], minlength=nvox) for c in range(3)], 1)
    mean = T1 / np.maximum(N[:, None], 1)
    # 体素总协方差 -> 最小特征向量 = 局部法向
    cov = np.zeros((nvox, 3, 3))
    for k, (i, j) in enumerate(PAIRS):
        v = np.bincount(pv, weights=s2[:, k], minlength=nvox) / np.maximum(N, 1) - mean[:, i] * mean[:, j]
        cov[:, i, j] = cov[:, j, i] = v
    evals, evecs = np.linalg.eigh(cov)
    nrm = evecs[:, :, 0]

    keep = (N >= a.min_pts) & (np.bincount(pv, minlength=nvox) >= a.min_frames)
    idx = np.where(keep)[0]
    print(f"入选体素 {len(idx)}（≥{a.min_pts} 点且 ≥{a.min_frames} 帧，无平面性筛选）")

    # 帧间三维散布（trace）：Σ_f |s1_f|²/c_f / N − |m̄|²
    q3 = ((s1 * s1).sum(1) / np.maximum(cnt, 1))
    between3 = np.bincount(pv, weights=q3, minlength=nvox) / np.maximum(N, 1) - (mean ** 2).sum(1)
    # 法向分量：d_f = n·s1_f/c_f（显式乘加，避免 einsum 走外积路径爆内存）
    d = (nrm[pv] * s1).sum(1) / np.maximum(cnt, 1)
    betweenN = (np.bincount(pv, weights=cnt * d * d, minlength=nvox) / np.maximum(N, 1)
                - np.where(keep, (nrm * mean).sum(1), 0.0) ** 2)
    b3 = np.sqrt(np.maximum(between3, 0.0)) * 100
    bN = np.sqrt(np.maximum(betweenN, 0.0)) * 100
    bT = np.sqrt(np.maximum(between3 - betweenN, 0.0)) * 100
    w = N[idx]

    def q(arr, label):
        x = arr[idx]
        o = np.argsort(x)
        xs, ws = x[o], w[o]
        cw = np.cumsum(ws) / ws.sum()
        p = [float(xs[np.searchsorted(cw, v)]) for v in (.5, .75, .9, .99)]
        print(f"  {label:12s} 中位={p[0]:.2f}  P75={p[1]:.2f}  P90={p[2]:.2f}  P99={p[3]:.2f} cm"
              f"   >5cm 加权占比={float((ws[xs > 5].sum() / ws.sum())):.3f}")
        return dict(med=p[0], p75=p[1], p90=p[2], p99=p[3], share_gt5=float((ws[xs > 5].sum() / ws.sum())))

    print("\n=== 跨帧散布（点数加权，无平面性筛选）===")
    res = {"between_3D": q(b3, "3D 总散布"),
           "between_normal": q(bN, "法向(分层)"),
           "between_tangent": q(bT, "切向(涂抹)")}
    res["within_frame"] = {}
    # 对照：帧内点散布（同一帧同一体素内）
    var_in = (s2[:, 0] + s2[:, 3] + s2[:, 5]) / np.maximum(cnt, 1) - q3
    win = np.sqrt(np.maximum(var_in, 0.0)) * 100
    xw = win[np.isfinite(win) & (cnt >= 10)]
    res["within_frame"] = dict(med=float(np.median(xw)), p90=float(np.percentile(xw, 90)))
    print(f"  {'帧内点散布':12s} 中位={np.median(xw):.2f}  P90={np.percentile(xw, 90):.2f} cm"
          f"（含传感器 footprint/表面粗糙）")

    print("\n=== 散布最大的 15 个体素位置（可对照招牌）===")
    top = idx[np.argsort(-b3[idx])[:15]]
    for t in top:
        print(f"  ({mean[t,0]:7.2f},{mean[t,1]:7.2f},{mean[t,2]:6.2f}) "
              f"3D={b3[t]:5.2f} 法向={bN[t]:5.2f} 切向={bT[t]:5.2f} cm  N={int(N[t])}")
    res["top_locations"] = [[round(float(mean[t, 0]), 2), round(float(mean[t, 1]), 2),
                             round(float(mean[t, 2]), 2), round(float(b3[t]), 2),
                             round(float(bN[t]), 2), round(float(bT[t]), 2), int(N[t])]
                            for t in top]

    # 上色 PCD：高散布体素一目了然
    lo, hi = np.percentile(b3[idx], [1, 99])
    g = np.clip((b3[idx] - lo) / max(hi - lo, 1e-9), 0, 1)
    rgb = np.stack([255 * np.clip(2 * g, 0, 1),
                    255 * np.clip(2 * (1 - g), 0, 1),
                    np.full(len(idx), 40)], 1).astype(np.uint8)
    write_pcd(os.path.join(a.out, f"spread_{int(a.edge*100)}cm.pcd"),
              mean[idx].astype(np.float32), rgb)
    print(f"\n上色体素图（红=高散布）：{a.out}/spread_{int(a.edge*100)}cm.pcd")

    # 招牌附近区域单独统计：用 top 位置附近的体素
    near = idx[np.linalg.norm(mean[idx] - mean[top[0]], axis=1) < 2.0]
    if len(near) > 20:
        print(f"最高散布点 2m 半径内 {len(near)} 体素：3D 中位={np.median(b3[near]):.2f}cm "
              f"P90={np.percentile(b3[near],90):.2f}cm")
        res["near_top"] = dict(n=int(len(near)), med=float(np.median(b3[near])),
                               p90=float(np.percentile(b3[near], 90)))

    with open(os.path.join(a.out, f"summary_{int(a.edge*100)}cm.json"), "w") as f:
        json.dump(dict(edge_m=a.edge, n_voxels=int(len(idx)), **res), f, indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
