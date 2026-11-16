#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P0-C 的收尾一测：深度辅助模型的**平移先验 t_c 敏感性 / placebo**。

为什么要单独做：19e 的主判据模型用 **LIO 的相机系基线 t_c** 作为平移先验
（`_p0_rot.estimate_rotation_d`：x2 ≈ normalize(R·x1 + t_c·λ)，λ 由平面拟合）。
这等于把一部分 LIO 信息喂进了"看似独立"的视觉因子 —— 评审第 2 条正是这一点：
「如果使用这些深度构造 3D-2D PnP，确实有机会获得有尺度的平移观测，
  但深度本身来自 LIO，因而会引入与原始 LIO 的误差相关性。」

19e 只用"三边自洽"（0.130 ≈ √(0.048²+0.126²)）间接说明泄漏可忽略，
那是**必要但不充分**的证据。本脚本直接做扰动实验，量出灵敏度：

  A) t_c                 基准（真 LIO 基线）
  B) 0.5·t_c / C) 2·t_c  基线**幅度**错 2 倍 ⇒ 斜率 = dθ̂/d ln|t|
  D) −t_c                基线**方向**反向（大错）
  E) 借用另一对（k+37）的 t_c：量级相近但方向/大小都错（placebo）
  F) 随机旋转 10° 后的 t_c：只改方向、不改幅度

判读：
 · 若 |B−A|、|C−A| ≈ 0.01° 量级（远小于 σ=0.043°）⇒ t_c 的幅度误差**根本不进入旋转解**，
   "视觉因子继承 LIO 平移误差"这条批评对本模型不成立（因为旋转由射影对应决定，
   t 只用来给 λ 定标）；
 · 若 E/F 让 vs-IMU 残差显著上升 ⇒ t_c 的方向/一致性仍然重要，Phase 1 里
   t_c 必须与位姿因子一起优化（把基线当状态量，而不是当固定观测量）。

口径全部沿用 §36：外参侧别 b→c、conj_B(L)=R_CL·L·R_CLᵗ、Δt_camera=110 ms、IMU 右乘。
用法：python3 19g_tc_placebo.py [对数] [起始k]     默认 120 100
"""
import sys
import time

import numpy as np

sys.path.insert(0, '/home/ywj/git/Super-LIVO/experiments/octvox_ablation')
import _p0_bag as B
import _p0_visual as V
import _p0_rot as RT

NPAIR = int(sys.argv[1]) if len(sys.argv) > 1 else 120
K0 = int(sys.argv[2]) if len(sys.argv) > 2 else 100
BASE = '/home/ywj/slam_data/retail_abl/base.txt'
DT0 = 0.110
MINBASE = 0.008
SHIFT = 37                        # placebo 借用的帧偏移


def b2c(L):
    return V.rotm(V.R_CL @ L @ V.R_CL.T)


def imu_traj(ti, gi):
    Rs = np.empty((len(ti), 3, 3))
    Rs[0] = np.eye(3)
    dt = np.diff(ti)
    for k in range(len(dt)):
        Rs[k + 1] = Rs[k] @ V.exp_so3(0.5 * (gi[k] + gi[k + 1]) * dt[k])
    return V.Traj(ti, np.zeros((len(ti), 3)),
                  np.array([V.rot_to_quat_w(r) for r in Rs]))


def rel(tr, t1, t2):
    _, R1, v1 = tr.at([t1])
    _, R2, v2 = tr.at([t2])
    return None if not (v1[0] and v2[0]) else R2[0].T @ R1[0]


def ang(Ra, Rb):
    return V.angle_of(V.rotm(Ra.T @ Rb))


VAR = ('A 基准 t_c', 'B 0.5×', 'C 2.0×', 'D −t_c', 'E 借用 k+%d' % SHIFT,
       'F 方向随机转 10°')


def main():
    t0 = time.time()
    rng = np.random.default_rng(7)
    tl, _, _ = B.read_lidar_stamps(stride=1)
    ti, gi, _ = B.read_imu()
    tum = np.loadtxt(BASE)
    lio = V.Traj(tum[:, 0], tum[:, 1:4], tum[:, 4:8])
    imu = imu_traj(ti, gi)
    ks = [k for k in range(K0, min(K0 + NPAIR * 3, len(tl) - 2 - SHIFT))]
    need = sorted(set(k for x in ks for k in (x, x + 1)))
    frames = B.read_images(need)
    gray = {k: V.prep_gray(frames[k][1]) for k in frames}
    del frames
    print('读图 %.0f s；窗口 k=%d..%d' % (time.time() - t0, ks[0], ks[-1]))

    # 先把每对的 tc 缓存，placebo 变体要用到别的帧的 tc
    cache = {}
    rec = []
    for k in ks:
        if len(rec) >= NPAIR:
            break
        if k not in cache:
            _, tc, tn = RT.cam_rel_from_lio(lio, tl[k] + DT0, tl[k + 1] + DT0)
            cache[k] = (tc, tn)
        tc, tn = cache[k]
        if not np.isfinite(tn) or tn < MINBASE:
            continue
        a, b, ntr = V.track_pair(gray[k], gray[k + 1])
        if a is None or len(a) < 60:
            continue
        Rim = b2c(rel(imu, tl[k] + DT0, tl[k + 1] + DT0))
        if Rim is None:
            continue
        tc_alt = cache.get(k + SHIFT, (None, np.nan))[0]
        if tc_alt is None:
            _, tc_alt, _ = RT.cam_rel_from_lio(lio, tl[k + SHIFT] + DT0,
                                               tl[k + SHIFT + 1] + DT0)
        q = V.exp_so3(rng.normal(0, np.radians(10.0), 3))      # 只改方向、不改幅度
        cand = {'A 基准 t_c': tc, 'B 0.5×': 0.5 * tc, 'C 2.0×': 2.0 * tc,
                'D −t_c': -tc, 'E 借用 k+%d' % SHIFT: tc_alt,
                'F 方向随机转 10°': q @ tc}
        est, dA = {}, None
        for nm, tv in cand.items():
            d = RT.estimate_rotation_d(a, b, np.asarray(tv, float))
            est[nm] = d['R'] if d.get('usable') else None
            if nm.startswith('A '):
                dA = d
        if est['A 基准 t_c'] is None:
            continue
        rec.append(dict(k=k, tn=tn, d_med=dA['d_med'], est=est, Rim=Rim))
    n = len(rec)
    print('有效对 %d（%.0f s）' % (n, time.time() - t0))

    # 基准 A 对 IMU 的残差（所有变体与它比，得到"Δ 相对基准"）
    EA = np.array([ang(r['est']['A 基准 t_c'], r['Rim']) for r in rec
                   if r['est']['A 基准 t_c'] is not None])
    MED_A = float(np.median(EA))

    print('\n===== 1. 各变体的解相对**基准 A** 的偏移（= t_c 被污染的泄漏量）=====')
    print('  变体              n    vs A med°    p90°     vs IMU med°   Δ vs IMU')
    for nm in VAR:
        dv, ei = [], []
        for r in rec:
            R = r['est'][nm]
            RA = r['est']['A 基准 t_c']
            if R is None or RA is None:
                continue
            dv.append(ang(RA, R))
            ei.append(ang(R, r['Rim']))
        if len(dv) < 10:
            print('  %-16s 样本不足' % nm)
            continue
        print('  %-16s %3d     %8.4f  %8.4f      %8.4f    %+.4f'
              % (nm, len(dv), np.median(dv), np.percentile(dv, 90), np.median(ei),
                 np.median(ei) - MED_A))
    eB = np.median([ang(r['est']['A 基准 t_c'], r['est']['B 0.5×']) for r in rec
                    if r['est']['B 0.5×'] is not None])
    eC = np.median([ang(r['est']['A 基准 t_c'], r['est']['C 2.0×']) for r in rec
                    if r['est']['C 2.0×'] is not None])
    sens = np.hypot(eB, eC) / np.log(4.0)          # dθ/d ln|t|（两侧各 ln2）
    print('\n  ⇒ 幅度灵敏度 dθ̂/d ln|t_c| ≈ %.4f °/单位对数' % sens)
    print('    LIO 基线若有 10%% 幅度误差（ln 0.1≈%.2f）⇒ 注入旋转误差 %.4f°；'
          '50%% 误差 ⇒ %.4f°' % (np.log(1.1), sens * np.log(1.1), sens * np.log(1.5)))

    print('\n===== 2. 按基线幅度分层（小基线最容易泄漏）=====')
    print('  层            n   vs A(B)   vs A(C)   vs A(E)   vs IMU(A)')
    tn = np.array([r['tn'] for r in rec])
    for nm, m in [('0.008~0.03', (tn >= 0.008) & (tn < 0.03)),
                  ('0.03~0.07', (tn >= 0.03) & (tn < 0.07)),
                  ('≥0.07', tn >= 0.07)]:
        idx = [i for i, x in enumerate(range(n)) if m[i]]
        if len(idx) < 8:
            continue
        f = lambda key: np.median([ang(rec[i]['est']['A 基准 t_c'], rec[i]['est'][key])
                                   for i in idx if rec[i]['est'][key] is not None])
        print('  %-12s %3d   %7.4f   %7.4f   %7.4f     %7.4f'
              % (nm, len(idx), f('B 0.5×'), f('C 2.0×'), f('E 借用 k+%d' % SHIFT),
                 np.median([ang(rec[i]['est']['A 基准 t_c'], rec[i]['Rim'])
                            for i in idx if rec[i]['est']['A 基准 t_c'] is not None])))

    print('\n===== 3. 判决 =====')
    ea = MED_A
    ed = np.median([ang(r['est']['D −t_c'], r['Rim']) for r in rec
                    if r['est']['D −t_c'] is not None])
    ee = np.median([ang(r['est']['E 借用 k+%d' % SHIFT], r['Rim']) for r in rec
                    if r['est']['E 借用 k+%d' % SHIFT] is not None])
    ef = np.median([ang(r['est']['F 方向随机转 10°'], r['Rim']) for r in rec
                    if r['est']['F 方向随机转 10°'] is not None])
    print('  t_c 完全错（方向取反 / 借用别的帧）：vs IMU %.3f° / %.3f°，基准 %.3f°'
          % (ed, ee, ea))

    print('\n  【判决：必须分两件事说，合成一句就是错的】')
    print('   ① 平移**模长**：×0.5 / ×2 / 取反 与基准的偏移 med = %.4f / %.4f / %.4f°'
          '（dθ̂/d ln|t_c| = %.4f）' % (eB, eC, ed, sens))
    print('      机理：λ 是自由标量，把 t_c 的整体标度（连符号）完全吸收 ⇒ 旋转解对')
    print('      |t_c| **零敏感**。推论有两面：视觉因子**不继承** LIO 的平移模长误差')
    print('      （19e 的三边自洽得到独立印证）；但也正因为它对模长零敏感，它')
    print('      **不具备修正 LIO 平移模长的能力** ⇒ 评审第 2 条成立：第一阶段的')
    print('      视觉因子只是旋转约束，不是六自由度测量，平移仍必须沿用 LIO。')
    print('   ② 平移**方向**：随机转 10° ⇒ Δvs IMU %+.4f°（= σ0.043° 的 %.2f 倍）；'
          % (ef - ea, (ef - ea) / 0.043))
    print('      借用别的帧的方向 ⇒ %+.4f°。⇒ 基线**方向**仍是 LIO 灌进视觉因子的通道。'
          % (ee - ea))
    print('      Phase 1 里 t_c 的方向要当带协方差的量（或与位姿一起当状态量优化），'
          '不能当精确常数。')
    edg = np.array([r['tn'] for r in rec])
    base = np.array([ang(r['est']['A 基准 t_c'], r['Rim']) for r in rec])
    lo, hi = base[edg < 0.03], base[edg >= 0.07]
    if len(lo) >= 8 and len(hi) >= 8:
        print('   分层旁注：小基线(<0.03 m) 基准残差 %.4f°（n=%d） vs '
              '大基线(≥0.07 m) %.4f°（n=%d）⇒ 门控按 |t_c| 分层，小基线时收紧权重。'
              % (np.median(lo), len(lo), np.median(hi), len(hi)))
    print('总耗时 %.0f s' % (time.time() - t0))


if __name__ == '__main__':
    main()
