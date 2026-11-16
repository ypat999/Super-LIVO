#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""合成真值：深度辅助（t 已知 + 单主导平面）模型能否把视差耦合的旋转轴偏差压到 0.1°？

19b 已证：裸 Procrustes 的偏差 ≈ |t|/d̄ 弧度，与像素噪声无关（本 bag ⇒ ≈0.6°）。
本脚本用**同一个前端**（噪声 0.1 px、250 点、网格化）对比三种估计器：
    P  纯旋转 Procrustes                （_p0_rot.estimate_rotation）
    D  深度辅助 6 参数 (δθ,q)，t=真值
    Dn 深度辅助 6 参数 (δθ,q)，t=真值×(1+7% 高斯)  ← 贴近 LIO 逐帧平移精度
    Dl 深度辅助，但 t 的**方向**取 LIO（同样含噪），模长取真值  ← 同 Dn，仅口径标注
另外检查 σ_θ 标定性：预测 σ（LM 处 JᵀJ 边缘协方差）vs 实测散布。

判据（P0-C 的实现能否继续）：Dn 的偏差中位 ≤ 0.15°。
"""
import sys
import numpy as np

sys.path.insert(0, '/home/ywj/git/Super-LIVO/experiments/octvox_ablation')
import _p0_visual as V
import _p0_rot as RT

rng = np.random.default_rng(2026)
NTRIAL = int(sys.argv[1]) if len(sys.argv) > 1 else 40
K = V.K_USE
NP = 250
NOISE = 0.1            # px


def scene(mode, dmed, npts=NP):
    u = rng.uniform(20, V.W - 20, npts)
    v = rng.uniform(20, V.H - 20, npts)
    dirn = np.linalg.inv(K) @ np.c_[u, v, np.ones(npts)].T
    dirn = (dirn / np.linalg.norm(dirn, axis=0)).T
    if mode == 'plane':
        n = np.array([0.0, 0.3, 0.95])
        n /= np.linalg.norm(n)
        d = dmed / (dirn @ n)
    elif mode == 'twoplane':                     # 近墙 + 远墙（真实街景）
        n1 = np.array([0.0, 0.2, 0.98])
        n1 /= np.linalg.norm(n1)
        n2 = np.array([0.6, 0.0, 0.8])
        n2 /= np.linalg.norm(n2)
        half = npts // 2
        d = np.r_[dmed / (dirn[:half] @ n1), 2.5 * dmed / (dirn[half:] @ n2)]
    else:
        d = rng.uniform(0.5 * dmed, 2.0 * dmed, npts)
    return dirn * d[:, None]


def gen(mode, dmed, tmag, terr):
    P = scene(mode, dmed)
    w = rng.normal(0, 1, 3)
    w /= np.linalg.norm(w)
    w *= np.radians(rng.uniform(0.5, 2.5))
    Rtrue = V.exp_so3(w)
    t = rng.normal(0, 1, 3)
    t /= np.linalg.norm(t)
    t *= tmag
    P2 = Rtrue @ P.T + t[:, None]
    x1 = (K @ P.T).T
    x1 = x1[:, :2] / x1[:, 2:3]
    x2 = (K @ P2).T
    x2 = x2[:, :2] / x2[:, 2:3]
    ok = (x1[:, 0] > 2) & (x1[:, 0] < V.W - 2) & (x1[:, 1] > 2) & (x1[:, 1] < V.H - 2) & \
         (x2[:, 0] > 2) & (x2[:, 0] < V.W - 2) & (x2[:, 1] > 2) & (x2[:, 1] < V.H - 2) & \
         (P2[2] > 0.3)
    a = x1[ok] + rng.normal(0, NOISE, (int(ok.sum()), 2))
    b = x2[ok] + rng.normal(0, NOISE, (int(ok.sum()), 2))
    tno = t * (1.0 + terr * rng.normal(0, 1, 3))
    return a, b, Rtrue, t, tno


def errdeg(R, Rtrue):
    return np.degrees(np.linalg.norm(V.log_so3(V.rotm(R).T @ Rtrue)))


def run(mode, dmed, tmag, terr=0.07, ntrial=NTRIAL):
    ep, ed, en, sg, sd_scat = [], [], [], [], []
    for _ in range(ntrial):
        a, b, Rtrue, t, tno = gen(mode, dmed, tmag, terr)
        if len(a) < 40:
            continue
        dp = RT.estimate_rotation(a, b)
        if dp['R'] is not None:
            ep.append(errdeg(dp['R'], Rtrue))
        dd = RT.estimate_rotation_d(a, b, t)
        dn = RT.estimate_rotation_d(a, b, tno)
        if dd['R'] is not None:
            ed.append(errdeg(dd['R'], Rtrue))
            sg.append(dd['sigma_rms_deg'])
        if dn['R'] is not None:
            en.append(errdeg(dn['R'], Rtrue))
            sd_scat.append(dn['sigma_rms_deg'])
    def st(x):
        x = np.array(x)
        return (np.median(x), np.percentile(x, 90)) if len(x) else (np.nan, np.nan)
    p, n = st(ep), st(en)
    s = np.median(np.array(sd_scat)) if sd_scat else np.nan
    print('  %-9s d̄=%4.1f |t|=%.2f  P med=%7.3f/%7.3f   Dt med=%7.3f/%7.3f   '
          'D(noisy t) med=%7.3f/%7.3f   σ̂=%7.4f  实测/σ̂=%5.1f×'
          % (mode, dmed, tmag, p[0], p[1], st(ed)[0], st(ed)[1], n[0], n[1], s,
             (n[0] / s) if (s and np.isfinite(s)) else np.nan))
    return n[0]


print('偏差单位 deg；P=纯旋转，Dt=t 精确，D(noisy t)=t 带 %.0f%% 噪声' % (100 * 0.07))
print('  实测/σ̂ 是 Dn 的“中位偏差 / 预测 1σ”，≈1~2 表示协方差标定合理\n')
for mode in ('plane', 'diverse', 'twoplane'):
    for dmed in (3.0, 6.0, 12.0):
        for tmag in (0.055, 0.10):
            run(mode, dmed, tmag)
    print('')
print('本 bag 实测：|dp|/帧 med=0.055 m；主墙 ~5 m ⇒ 看 d̄=6.0 / |t|=0.06 行')
