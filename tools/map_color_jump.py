#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""最终地图的点级颜色跳变参照（非栅格口径，无 z-buffer）：
每图随机 40k 查询点，取空间 k=8 邻域（KDTree 用均匀子样本控制内存），
统计 |lum_self - median(lum_neighbors)| > 40 的比例 + kNN 中位距离。
三图：sl_map(中位数融合) / e1_map(纯追加) / 官方 retail 静扫上色图。
"""
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
from blackframe_voxel_replay import read_pcd_xyzrgb, rgb_of, lum_of

MAPS = [
    ("sl_map_median", "/home/ywj/slam_data/singleframe/sl_map/super_livo_visual_a_colored.pcd", True),
    ("e1_map_append", "/home/ywj/slam_data/e1_append/e1_map/super_livo_visual_a_colored.pcd", True),
    ("fl2_retail_static", "/home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb.pcd", False),
]
rng = np.random.default_rng(3)
for nm, p, is_pk in MAPS:
    P, pk = read_pcd_xyzrgb(p)
    L = lum_of(pk)
    keep = L > 3.0                      # 黑点（未上色）不参与
    P, L = P[keep], L[keep]
    sub = rng.choice(len(P), size=min(len(P), 3_000_000), replace=False)
    tree = cKDTree(P[sub])
    q = rng.choice(len(P), size=40000, replace=False)
    d, i = tree.query(P[q], k=8)
    ln = L[sub[i]]                      # (40k,8)
    jump = np.abs(L[q] - np.median(ln, 1)) > 40.0
    # 排除近空邻域（kNN 距离 >0.5m 的点：稀疏区不可比）
    okd = d[:, 4] < 0.5
    print(f"{nm}: pts={len(P):,}  kNN4中位={np.median(d[:,4]):.3f}m  "
          f"跳变率={jump[okd].mean()*100:.2f}%  "
          f"(lum极差>80: {np.ptp(ln,1)[okd].mean():.0f}±"
          f"{np.ptp(ln,1)[okd].std():.0f})")
