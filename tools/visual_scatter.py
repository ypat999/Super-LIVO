#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉可辨离散度：多帧厚度 vs 单帧厚度（对应肉眼看到的"离散半径"）。

为什么不用质心散布：5cm 体素内每帧只有几十个点，跨帧质心散布被质心自身的
采样噪声（~1-2cm）淹没，分布窄到 P50=1.93/P99=2.36，无法分辨真实重影。

本脚本改用**厚度**，不涉及任何平均：
  对每个 0.5m 单元，沿该单元局部法向取深度 d：
    thick_multi  = P95(d) − P5(d)  用该单元全部帧的点     ← 肉眼看到的糊团尺寸
    thick_single = 各帧单独算 P95−P5 后取中位             ← 单帧本身的尺寸
    ghost        = thick_multi − thick_single             ← 重影净贡献
非平面/稀疏区域不筛选，只要求单元点数足够，避免把招牌字排除掉。

用法：python3 tools/visual_scatter.py [--cell 0.5] [--center X Y Z] [--radius R]
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame_wall_offset import load
from frame_id_color_map import write_pcd
from voxel_attribution import pack16

PAIRS = ((0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="/home/ywj/slam_data/pcd/retail_framecolor/PCD")
    ap.add_argument("--out", default="/home/ywj/slam_data/pcd/retail_framecolor/spread")
    ap.add_argument("--cell", type=float, default=0.5)
    ap.add_argument("--min-pts", type=int, default=500)
    ap.add_argument("--min-per-frame", type=int, default=40)
    ap.add_argument("--center", type=float, nargs=3, default=None)
    ap.add_argument("--radius", type=float, default=2.0)
    ap.add_argument("--max-lam", type=float, default=4e-4,
                    help="单元最小特征值上限 (m²)；4e-4 ≈ 单帧厚度 std 2cm，"
                         "只保留真正成面的单元，避免非平面单元抬高单帧厚度基线")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    P, fr, pose, names = load(a.src)
    Pd = P.astype(np.float64)
    cell = np.floor(P / a.cell).astype(np.int32)
    uniq, inv = np.unique(pack16(cell), return_inverse=True)
    ncell = len(uniq)
    n = np.bincount(inv, minlength=ncell).astype(np.float64)
    cent = np.stack([np.bincount(inv, weights=Pd[:, c], minlength=ncell) for c in range(3)], 1) / \
        np.maximum(n[:, None], 1)
    cov = np.zeros((ncell, 3, 3))
    for k, (i, j) in enumerate(PAIRS):
        v = np.bincount(inv, weights=Pd[:, i] * Pd[:, j], minlength=ncell) / np.maximum(n, 1) \
            - cent[:, i] * cent[:, j]
        cov[:, i, j] = cov[:, j, i] = v
    evals, evecs = np.linalg.eigh(cov)
    nrm = evecs[:, :, 0]
    keep = np.where((n >= a.min_pts) & (evals[:, 0] <= a.max_lam))[0]
    print(f"单元 {ncell}，入选 {len(keep)}（≥{a.min_pts} 点且 λmin≤{a.max_lam:g} m²）")

    d = ((Pd - cent[inv]) * nrm[inv]).sum(1)
    order = np.argsort(inv, kind="stable")
    inv_s, d_s, f_s = inv[order], d[order], fr[order]
    starts = np.searchsorted(inv_s, np.arange(ncell + 1))

    multi, single, top = [], [], []
    for ci in keep:
        seg = d_s[starts[ci]:starts[ci + 1]]
        fs = f_s[starts[ci]:starts[ci + 1]]
        tm = np.percentile(seg, 95) - np.percentile(seg, 5)
        per = []
        for f in np.unique(fs):
            s = seg[fs == f]
            if len(s) >= a.min_per_frame:
                per.append(np.percentile(s, 95) - np.percentile(s, 5))
        if len(per) < 5:
            continue
        multi.append(tm * 100)
        single.append(np.median(per) * 100)
        top.append(ci)
    multi = np.array(multi); single = np.array(single)
    ghost = np.maximum(multi - single, 0.0)
    w = n[top]

    def q(arr, lab):
        o = np.argsort(arr); xs, ws = arr[o], w[o]
        cw = np.cumsum(ws) / ws.sum()
        p = [float(xs[np.searchsorted(cw, f)]) for f in (.5, .75, .9, .99)]
        print(f"  {lab:14s} 中位={p[0]:.2f}  P75={p[1]:.2f}  P90={p[2]:.2f}  P99={p[3]:.2f} cm"
              f"   >5cm 占比={float(ws[xs > 5].sum()/ws.sum()):.3f}")
        return dict(med=p[0], p75=p[1], p90=p[2], p99=p[3], share_gt5=float(ws[xs > 5].sum()/ws.sum()))

    print("\n=== 视觉可辨离散度（P95−P5 沿局部法向）===")
    res = {"thickness_multi_frame": q(multi, "多帧合并厚度"),
           "thickness_single_frame": q(single, "单帧厚度"),
           "ghost_extra": q(ghost, "重影净增量")}
    print(f"  多帧/单帧厚度比：中位={np.median(multi/np.maximum(single,1e-9)):.2f}")

    print("\n=== 最糊的 15 个单元 ===")
    idx = np.argsort(-multi)[:15]
    for i in idx:
        ci = top[i]
        print(f"  ({cent[ci,0]:7.2f},{cent[ci,1]:7.2f},{cent[ci,2]:6.2f}) "
              f"多帧={multi[i]:5.2f} 单帧={single[i]:5.2f} 净增={ghost[i]:5.2f}cm N={int(n[ci])}")
    res["worst"] = [[round(float(cent[top[i], 0]), 2), round(float(cent[top[i], 1]), 2),
                     round(float(cent[top[i], 2]), 2), round(float(multi[i]), 2),
                     round(float(single[i]), 2), round(float(ghost[i]), 2)] for i in idx]

    lo, hi = np.percentile(multi, [1, 99])
    g = np.clip((multi - lo) / max(hi - lo, 1e-9), 0, 1)
    rgb = np.stack([255 * np.clip(2 * g, 0, 1), 255 * np.clip(2 * (1 - g), 0, 1),
                    np.full(len(multi), 40)], 1).astype(np.uint8)
    write_pcd(os.path.join(a.out, "visual_scatter.pcd"), cent[top].astype(np.float32), rgb)
    print(f"\n上色单元图（红=最糊）：{a.out}/visual_scatter.pcd")

    if a.center:
        c = np.array(a.center)
        m = np.linalg.norm(cent[top] - c, axis=1) < a.radius
        if m.sum() > 5:
            print(f"\n指定区域（{a.radius}m 内 {m.sum()} 单元）：多帧中位={np.median(multi[m]):.2f}cm "
                  f"P90={np.percentile(multi[m],90):.2f}cm | 单帧中位={np.median(single[m]):.2f}cm "
                  f"| 净增中位={np.median(ghost[m]):.2f}cm")
            res["roi"] = dict(n=int(m.sum()), multi_med=float(np.median(multi[m])),
                              multi_p90=float(np.percentile(multi[m], 90)),
                              single_med=float(np.median(single[m])),
                              ghost_med=float(np.median(ghost[m])))

    with open(os.path.join(a.out, "visual_scatter.json"), "w") as f:
        json.dump(dict(cell_m=a.cell, n_cells=int(len(multi)), **res), f, indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
