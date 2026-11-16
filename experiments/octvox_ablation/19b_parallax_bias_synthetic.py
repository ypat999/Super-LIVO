#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""合成真值实验：纯旋转(Procrustes)模型在**存在视差**时的旋转轴偏差有多大？

H = K(R + t nᵀ/d)K⁻¹ 的 8 自由度 = 3(旋转) + 5(t,n,d 除 1 尺度)。
⇒ 单一平面场景下旋转**不可辨识**；真实场景深度多样 ⇒ 偏差量级 ~ |t|/d̄。
本脚本用真值直接量出这个偏差，判断实测 1.29° 残差是否由此解释。
"""
import sys
import numpy as np
sys.path.insert(0, '/home/ywj/git/Super-LIVO/experiments/octvox_ablation')
import _p0_visual as V
import _p0_rot as RT

rng = np.random.default_rng(7)
K = V.K_USE
NP = 250
NTRIAL = 300


def scene(mode, dmed):
    """生成相机系 3D 点（像素布均匀近似：按视线反投）。"""
    u = rng.uniform(30, V.W - 30, NP)
    v = rng.uniform(30, V.H - 30, NP)
    dirn = np.linalg.inv(K) @ np.c_[u, v, np.ones(NP)].T
    dirn = dirn.T
    dirn /= np.linalg.norm(dirn, axis=1, keepdims=True)
    if mode == 'plane':                       # 一堵斜墙：法向 30° 偏转
        n = np.array([0.0, 0.3, 0.95])
        n /= np.linalg.norm(n)
        d = dmed / (dirn @ n)
    else:                                     # 深度分散
        d = rng.uniform(0.5 * dmed, 2.0 * dmed, NP)
    return dirn * d[:, None]


print('  场景          |t|(m)   d̄(m)   真值旋转   纯旋转模型偏差 med   90%   (deg)')
for mode in ('plane', 'diverse'):
    for dmed in (3.0, 6.0, 12.0):
        for tmag in (0.03, 0.10, 0.25):
            errs = []
            for _ in range(NTRIAL // 3):
                P = scene(mode, dmed)
                w = rng.normal(0, 1, 3)
                w /= np.linalg.norm(w)
                w *= np.radians(rng.uniform(0.5, 2.5))       # 真值 0.5–2.5°
                Rtrue = V.exp_so3(w)
                t = rng.normal(0, 1, 3)
                t /= np.linalg.norm(t)
                t *= tmag
                P2 = Rtrue @ P.T + t[:, None]
                x1 = (K @ P.T).T
                x1 = x1[:, :2] / x1[:, 2:3]
                x2 = (K @ P2).T
                x2 = x2[:, :2] / x2[:, 2:3]
                ok = (x1[:, 0] > 2) & (x1[:, 0] < V.W - 2) & (x1[:, 1] > 2) & \
                     (x1[:, 1] < V.H - 2) & (x2[:, 0] > 2) & (x2[:, 0] < V.W - 2) & \
                     (x2[:, 1] > 2) & (x2[:, 1] < V.H - 2) & (P2[2] > 0.3)
                if ok.sum() < 40:
                    continue
                a, b = x1[ok] + rng.normal(0, 0.1, (ok.sum(), 2)), \
                       x2[ok] + rng.normal(0, 0.1, (ok.sum(), 2))
                Rh = RT.procrustes_rot(RT._bearing(a), RT._bearing(b))
                errs.append(np.degrees(np.linalg.norm(V.log_so3(Rh.T @ Rtrue))))
            e = np.array(errs)
            print('  %-12s %6.2f %6.1f  0.5–2.5°     %8.3f  %8.3f'
                  % (mode, tmag, dmed, np.median(e), np.percentile(e, 90)))
