#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P0-D：图像旋转通道的**重复估计一致性**（评审第五条点名要求的实验）。

19e 已经证明"对 IMU 的偏差"很小（深度辅助 med 0.048°，其中随机 0.043°/定偏差 0.018°），
但那个数是"一次估计 vs 一个参照"，它无法回答两个关键问题：

 (Q1) 同一对图像换一批特征点，答案会不会变？ —— 即**重复性 / 内部散布**。
      若散布 ≫ 模型自报的 σ̂（JᵀJ 协方差），说明 σ̂ 被低估，Phase 1 的加权就不能用它。
 (Q2) 散布随特征数 N 的衰减是不是 1/√N？
      是 ⇒ 真的是"独立观测累加"，加特征有效；
      不是（趋于平台）⇒ 存在**与 N 无关的系统项**（视差耦合偏差 / 动态物 / 畸变残差），
      加特征救不了，只能靠深度源或标定。

对应评审的 5 条做法：
 1. 同一对图像用不同特征子集（50/100/200/300/全）bootstrap 重复求解；
 2. 按图像网格（1x1/2x2/4x4/6x6 分格）分区求解，看空间分布对结果的影响；
 3. LK 正/反向 + RANSAC 联合剔除（track_pair 已做前后向一致性；本脚本额外做
    正解 R̂12 与反解 R̂21 的逆一致，并统计子集下是否劣化）；
 4. 记录 R̂ 的**分布**（轴向 std / 模长 P50-P90）、**协方差**（模型 σ̂ vs 实测散布之比）、
    **内点空间覆盖率**（4x4 格内 ≥2 点的格子占比），而不是只报内点率；
 5. 与 IMU 角速度积分（200 Hz，右乘）独立对照，按深度分层看偏差来源。

口径全部沿用 §36 订正后的：外参侧别 b→c ⇒ conj_B(L)=R_CL·L·R_CLᵗ；Δt_camera=110 ms；
IMU 右乘积分；模型=深度辅助(t=LIO)（19e 的主判据模型 JDM）。

用法：python3 19f_repeat_precision.py [对数] [起始k] [每档重复次数]   默认 150 100 4
判决（写死在末尾）：
  σ_full（split-half 外推到全 n_pts 的**求解器+跟踪**随机 σ）≤0.10° 且 σ/σ̂ ≤3
      ⇒ 重复性达标、协方差可信，可进 P0-E 定权；
  σ/σ̂ >3 ⇒ σ̂ 被低估，Phase 1 必须改用实测散布定权；
  σ_full >0.30° ⇒ 降级（与 19e 结论矛盾，须回查前端）。
  另看 σ·√m 是否随 m 上升：上升 ⇒ 存在与 N 无关的**相关误差**（这才是 vs-IMU 残差的大头）。
"""
import sys
import time

import numpy as np

sys.path.insert(0, '/home/ywj/git/Super-LIVO/experiments/octvox_ablation')
import _p0_bag as B
import _p0_visual as V
import _p0_rot as RT

NPAIR = int(sys.argv[1]) if len(sys.argv) > 1 else 150
K0 = int(sys.argv[2]) if len(sys.argv) > 2 else 100
NREP = int(sys.argv[3]) if len(sys.argv) > 3 else 4
BASE = '/home/ywj/slam_data/retail_abl/base.txt'
DT0 = 0.110                     # probe_ext3 + 19e 双重复现的 Δt_camera
SIZES = (50, 100, 200)          # bootstrap 子集规模（相对全图解的位移，评审第1条）
CAP = 0.85                      # 子集规模超过 n_points*CAP 就不测（子集≈全集 ⇒ 位移恒 0，无意义）
HALVES = (25, 50, 100, 150)     # 两半互差规模 m（每半 m 点，需 2m ≤ n）—— 给出**绝对** σ
CELLS = (2, 4, 6)               # 图像网格分区数（评审第2条）
MINPTS = 30                     # 子集/单格至少多少点才解
MINBASE = 0.008                 # 相机基线短于此 ⇒ 视差不足，深度辅助退化
RNG = np.random.default_rng(0)


def b2c(L):
    """body 系区间相对旋转 -> 相机系（V.R_CL 实测 b→c）。"""
    return V.rotm(V.R_CL @ L @ V.R_CL.T)


def imu_traj(ti, gi):
    """陀螺 body 系 ⇒ 右乘递推 R(k+1)=R(k)·Exp(ω̄Δt)，得到"只有旋转"的轨迹。"""
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


def dev_deg(Ra, Rb):
    """两个旋转之差，返回 (模长°, rotvec° 3 维)。"""
    e = np.degrees(V.log_so3(V.rotm(Ra.T @ Rb)))
    return float(np.linalg.norm(e)), e


def cell_of(pts, ncell):
    """像素点 -> (ncell x ncell) 格子编号，越界夹紧。"""
    xs = np.clip((pts[:, 0] * ncell / V.W).astype(int), 0, ncell - 1)
    ys = np.clip((pts[:, 1] * ncell / V.H).astype(int), 0, ncell - 1)
    return ys * ncell + xs


def cov4(pts):
    """4x4 格「≥2 点」的占比：内点空间覆盖率。"""
    c = np.zeros(16, int)
    idx = np.clip((pts[:, 1] * 4 / V.H).astype(int), 0, 3) * 4 + \
        np.clip((pts[:, 0] * 4 / V.W).astype(int), 0, 3)
    np.add.at(c, idx, 1)
    return float((c >= 2).mean()), int((c >= 2).sum())


def main():
    t0 = time.time()
    tl, _, _ = B.read_lidar_stamps(stride=1)
    ti, gi, _ = B.read_imu()
    tum = np.loadtxt(BASE)
    lio = V.Traj(tum[:, 0], tum[:, 1:4], tum[:, 4:8])
    imu = imu_traj(ti, gi)
    ks = [k for k in range(K0, min(K0 + NPAIR * 3, len(tl) - 2))][:NPAIR * 3]
    need = sorted(set(k for x in ks for k in (x, x + 1)))
    frames = B.read_images(need)
    gray = {k: V.prep_gray(frames[k][1]) for k in frames}
    del frames
    print('取样窗口 k=%d..；读图 %.0f s' % (ks[0], time.time() - t0))

    rec = []          # 每对一条记录
    tk = time.time()
    for k in ks:
        if len(rec) >= NPAIR:
            break
        a, b, ntr = V.track_pair(gray[k], gray[k + 1])
        if a is None or len(a) < 2 * MINPTS:
            continue
        _, tc, tn = RT.cam_rel_from_lio(lio, tl[k] + DT0, tl[k + 1] + DT0)
        if not np.isfinite(tn) or tn < MINBASE:
            continue
        Rim = b2c(rel(imu, tl[k] + DT0, tl[k + 1] + DT0))
        if Rim is None:
            continue
        full = RT.estimate_rotation_d(a, b, tc)
        if not full.get('usable'):
            continue
        Rf = full['R']
        # 反解：同一批对应点几何角色互换（源=k+1、目标=k），
        # t 必须按**反向时间**重算（tc 的符号取反不够，还差一个 R_rel 的共轭）。
        _, tc_r, tn_r = RT.cam_rel_from_lio(lio, tl[k + 1] + DT0, tl[k] + DT0)
        rev = RT.estimate_rotation_d(b, a, tc_r) if np.isfinite(tn_r) else {}
        r = dict(k=k, n=len(a), n_tracked=int(ntr), th=full['th_deg'],
                 d_med=full['d_med'], t_norm=tn, n_inl=full['n_inl'],
                 px_med=full['px_med'], cov=full['coverage'],
                 sig=full['sigma_deg'], sigrms=full['sigma_rms_deg'])
        r['inv'] = dev_deg(Rf, V.rotm(rev['R'].T))[0] if rev.get('usable') else np.nan
        r['e_full'] = dev_deg(Rf, Rim)[0]
        r['ef_vec'] = dev_deg(Rf, Rim)[1]
        # ---- 1. 特征子集 bootstrap：同一对图像、不同点集（相对全图解的位移）----
        sub = {}
        for s in SIZES:
            m = min(s, len(a))
            if m < MINPTS or m > CAP * len(a):
                continue                       # 子集≈全集 ⇒ 位移恒 0，无意义
            dv, sg = [], []
            for _ in range(NREP):
                sel = RNG.permutation(len(a))[:m]
                d = RT.estimate_rotation_d(a[sel], b[sel], tc)
                if not d.get('usable'):
                    continue
                dv.append(dev_deg(d['R'], Rf)[0])          # 相对全集解的散布
                sg.append(d['sigma_rms_deg'])
            if dv:
                sub[s] = dict(p50=float(np.median(dv)), p90=float(np.percentile(dv, 90)),
                              pred=float(np.median(sg)), n=len(dv), m=m)
        r['sub'] = sub
        # ---- 1b. 两半互差（disjoint split-half）：给**绝对** σ，不受"相对全集"收缩偏差影响
        #   两半点集不相交 ⇒ 两个解近乎独立 ⇒ ∠(R_A·R_Bᵗ) ≈ √2·σ(m)；σ(m)·√m 恒定 ⇔ 1/√N
        hal = {}
        for mm in HALVES:
            if 2 * mm > len(a) or mm < 20:
                continue
            dv, sg = [], []
            for _ in range(NREP):
                pm = RNG.permutation(len(a))
                dA = RT.estimate_rotation_d(a[pm[:mm]], b[pm[:mm]], tc)
                dB = RT.estimate_rotation_d(a[pm[mm:2 * mm]], b[pm[mm:2 * mm]], tc)
                if not (dA.get('usable') and dB.get('usable')):
                    continue
                dv.append(dev_deg(dA['R'], dB['R'])[0])
                sg.append(0.5 * (dA['sigma_rms_deg'] + dB['sigma_rms_deg']))
            if dv:
                hal[mm] = dict(dev=float(np.median(dv)), sig=float(np.median(dv)) / np.sqrt(2),
                               pred=float(np.median(sg)), n=len(dv))
        r['hal'] = hal
        # ---- 2. 网格分区：每格单独解（空间分布敏感性）----
        cellmax = {}
        for nc in CELLS:
            if nc == 1:
                continue
            ids = cell_of(a, nc)
            devs, cnt = [], 0
            for c in np.unique(ids):
                sel = np.where(ids == c)[0]
                if len(sel) < MINPTS:
                    continue
                d = RT.estimate_rotation_d(a[sel], b[sel], tc)
                if not d.get('usable'):
                    continue
                devs.append(dev_deg(d['R'], Rf)[0])
                cnt += 1
            if devs:
                cellmax[nc] = dict(p50=float(np.median(devs)),
                                   p90=float(np.percentile(devs, 90)),
                                   max=float(np.max(devs)), nc=cnt)
        r['cell'] = cellmax
        rec.append(r)
    print('处理 %d 对 %.0f s（跟踪+全/反/子集/分格 ≈ %d 次求解）'
          % (len(rec), time.time() - tk, len(rec) * 40))

    n = len(rec)
    print('\n===== 1. 全图单解（基准）=====')
    print('  |R̂| med=%.3f°  n_pts med=%d  n_inl med=%d  |t| med=%.4f m  '
          'd̄ med=%.1f m  px med=%.3f' %
          (np.median([r['th'] for r in rec]), np.median([r['n'] for r in rec]),
           int(np.median([r['n_inl'] for r in rec])),
           np.median([r['t_norm'] for r in rec]), np.median([r['d_med'] for r in rec]),
           np.median([r['px_med'] for r in rec])))
    print('  内点空间覆盖率(4x4,≥2) med=%.2f 分格数 med=%d  '
          '（评审第4条：不只看内点率）' %
          (np.median([r['cov'] for r in rec]),
           int(round(np.median([r['cov'] for r in rec]) * 16))))
    print('  逆一致性 ∠(R̂12·R̂21) med=%.4f° ⇒ σ≈%.4f°' %
          (np.median([r['inv'] for r in rec if np.isfinite(r['inv'])]),
           np.median([r['inv'] for r in rec if np.isfinite(r['inv'])]) / np.sqrt(2)))
    print('    （注意：正反两解共用**同一批跟踪点**，LK 噪声相关 ⇒ 此值必然 ≤ 19e 的'
          '独立跟踪口径 0.005°，它只反映求解器本身的确定性）')
    ev = np.array([r['ef_vec'] for r in rec])
    print('  全图解 vs IMU：总 med=%.3f°  定偏差 μ=[%+.3f,%+.3f,%+.3f]° |μ|=%.3f°  '
          '随机轴向 σ=[%.3f,%.3f,%.3f]°' %
          (np.median([r['e_full'] for r in rec]), ev.mean(0)[0], ev.mean(0)[1],
           ev.mean(0)[2], np.linalg.norm(ev.mean(0)), ev.std(0)[0], ev.std(0)[1],
           ev.std(0)[2]))
    print('  模型自报 σ̂(JᵀJ)：med=%.4f°  p90=%.4f°  ← 对比下面的实测散布，检验是否被低估' %
          (np.median([r['sigrms'] for r in rec]),
           np.percentile([r['sigrms'] for r in rec], 90)))

    n_pts = float(np.median([r['n'] for r in rec]))
    print('\n===== 2a. 绝对重复性：disjoint split-half（两半各 m 点，点集不相交）=====')
    print('   m/半   n对   两半互差med    σ(m)=互差/√2    模型σ̂      σ/σ̂    σ·√m')
    H = {}
    for mm in HALVES:
        dv = [r['hal'][mm]['dev'] for r in rec if mm in r['hal']]
        pd = [r['hal'][mm]['pred'] for r in rec if mm in r['hal']]
        if len(dv) < 10:
            print('  %5d  %3d  —— 样本不足（需 2m ≤ n_pts≈%d）' % (mm, len(dv), n_pts))
            continue
        dvm = float(np.median(dv))
        sg = dvm / np.sqrt(2)
        pm = float(np.median(pd))
        H[mm] = dict(dev=dvm, sig=sg, pred=pm, ratio=sg / max(pm, 1e-9),
                     cnst=sg * np.sqrt(mm), n=len(dv))
        print('  %5d  %3d    %8.4f       %8.4f      %8.4f   %6.2f  %7.4f'
              % (mm, len(dv), dvm, sg, pm, H[mm]['ratio'], H[mm]['cnst']))
    if H:
        cc = [H[k]['cnst'] for k in sorted(H)]
        C = float(np.median(cc))
        SIG_FULL = C / np.sqrt(n_pts)
        print('  1/√N：σ·√m = %s ⇒ 末/首 %.2f（1.0=纯随机独立累加；>1.3=有 N 压不掉的相关项）'
              % (', '.join('%.4f' % v for v in cc), cc[-1] / max(cc[0], 1e-9)))
        print('  ⇒ 外推到全 n_pts≈%d 的**求解器自身** σ_full = %.4f°' % (n_pts, SIG_FULL))
        print('  ⇒ 模型 σ̂(全图) med=%.4f° vs 实测 σ_full=%.4f° ⇒ 比值 %.2f'
              % (np.median([r['sigrms'] for r in rec]), SIG_FULL,
                 SIG_FULL / max(np.median([r['sigrms'] for r in rec]), 1e-9)))
        rdm = np.median([r['e_full'] for r in rec])
        print('  ⇒ 19e 的 vs-IMU 随机 med=%.3f°：其中 %.0f%% 是**与 N 无关的相关误差**'
              '（动态物/离面点/畸变残差/LIO 深度误差），只靠加特征压不掉'
              % (rdm, 100 * max(0.0, np.sqrt(max(rdm ** 2 - SIG_FULL ** 2, 0.0)) / rdm)))
    else:
        SIG_FULL = np.nan

    print('\n===== 2b. bootstrap 子集相对全图解的位移（条件口径，偏小，仅看趋势）=====')
    print('   N子集  n_pts上限  n对   位移P50   位移P90    模型σ̂    位移/σ̂')
    S = {}
    for s in SIZES:
        p50 = [r['sub'][s]['p50'] for r in rec if s in r['sub']]
        p90 = [r['sub'][s]['p90'] for r in rec if s in r['sub']]
        pred = [r['sub'][s]['pred'] for r in rec if s in r['sub']]
        if len(p50) < 10:
            print('  %5d      ——     %3d  —— 样本不足' % (s, len(p50)))
            continue
        m50, mp = float(np.median(p50)), float(np.median(pred))
        S[s] = (m50, mp)
        print('  %5d   %6d   %3d   %7.4f   %7.4f  %8.4f   %6.2f'
              % (s, int(min(s, CAP * n_pts)), len(p50), m50,
                 float(np.median(p90)), mp, m50 / max(mp, 1e-9)))

    print('\n===== 3. 图像网格分区：单格解 vs 全图解（空间分布敏感性）=====')
    print('   格数  单格数/对  偏差P50   偏差P90   最差max   ⇒ 含义')
    for nc in CELLS:
        if nc == 1:
            continue
        d50 = [r['cell'][nc]['p50'] for r in rec if nc in r['cell']]
        d90 = [r['cell'][nc]['p90'] for r in rec if nc in r['cell']]
        ncs = [r['cell'][nc]['nc'] for r in rec if nc in r['cell']]
        if len(d50) < 10:
            print('  %d²x%d²  样本不足(%d)' % (nc, nc, len(d50)))
            continue
        print('  %2dx%2d  %6.1f   %7.4f   %7.4f   %7.3f   '
              '每格约 %d 点' % (nc, nc, np.mean(ncs), float(np.median(d50)),
                               float(np.median(d90)),
                               float(np.max([r['cell'][nc]['max'] for r in rec
                                             if nc in r['cell']])),
                               int(np.median([r['n'] for r in rec]) / (nc * nc))))

    print('\n===== 4. 按深度/基线分层（偏差来源：随机噪声 vs 视差耦合）=====')
    print('   层                n   两半互差(100)  全解vsIMU   |t|/d̄')
    hk = max(H) if H else None
    arr = np.array([[r['d_med'], r['t_norm'], r['e_full'],
                     r['hal'][hk]['dev'] if (hk and hk in r['hal']) else np.nan]
                    for r in rec])
    lay = [('d̄<6 m', arr[:, 0] < 6), ('6~12 m', (arr[:, 0] >= 6) & (arr[:, 0] < 12)),
           ('d̄>12 m', arr[:, 0] >= 12),
           ('|t|<0.04', arr[:, 1] < 0.04), ('|t|≥0.09', arr[:, 1] >= 0.09)]
    for nm, msk in lay:
        s = arr[msk]
        if len(s) < 8:
            continue
        print('   %-16s %3d      %7.4f     %7.3f   %7.4f' %
              (nm, len(s), np.nanmedian(s[:, 3]), np.median(s[:, 2]),
               np.median(s[:, 1] / np.maximum(s[:, 0], 0.5))))
    print('  （若 vsIMU 随 d̄ 变小 / |t| 变大而上升，而两半互差不变 ⇒ 上升来自**相关偏差**'
          '而非噪声）')

    # ---------------- 判决 ----------------
    print('\n===== 5. 判决（P0-D）=====')
    if not H:
        print('  split-half 无足够样本，P0-D 未成立，不得开 Phase 1。')
        return
    ratio = float(np.median([H[k]['ratio'] for k in sorted(H)]))
    m50 = S[SIZES[0]][0] if SIZES[0] in S else np.nan
    print('  绝对重复性：σ_full（外推到 n_pts≈%d）= %.4f°' % (n_pts, SIG_FULL))
    print('  bootstrap 相对全集位移 P50(N=%d) = %.4f°（条件口径，作下界参考）'
          % (SIZES[0], m50))
    print('  协方差诚实度：各规模 σ(m)/模型σ̂ 中位 = %.2f' % ratio)
    cc = [H[k]['cnst'] for k in sorted(H)]
    tail = cc[-1] / max(cc[0], 1e-9)
    if SIG_FULL > 0.30:
        print('  ⇒ 降级：σ_full=%.3f° 超 0.30° 档，与 19e 结论矛盾，回查前端。' % SIG_FULL)
    elif SIG_FULL > 0.10:
        print('  ⇒ 条件通过区：σ_full=%.3f° ∈(0.10,0.30]，Phase 1 不得直接开。' % SIG_FULL)
    else:
        print('  ⇒ 重复性达标：σ_full=%.4f° ≤0.10°，且远小于 vs-IMU 的 %.3f°'
              '⇒ **求解器/跟踪的随机性不是瓶颈**，瓶颈在相关误差。'
              % (SIG_FULL, np.median([r['e_full'] for r in rec])))
    if tail > 1.3:
        print('  ⇒ σ·√m 随 m 上升（%.2f）：存在**加特征压不掉的相关项**（离面点/动态物/'
              '畸变残差），Phase 1 必须靠深度源 + 门控，而不是堆特征数。' % tail)
    else:
        print('  ⇒ σ·√m 近似恒定（%.2f）：随机项符合 1/√N，加特征有效。' % tail)
    if ratio > 3.0:
        print('  ⇒ 模型协方差 σ̂ 偏乐观 %.1f 倍：Σ_vis,k=(JᵀJ)⁻¹ 可用，但需乘 '
              '%.1f 的保守系数，或改用实测 σ_full 的定标表。' % (ratio, ratio))
    else:
        print('  ⇒ 模型协方差 σ̂ 可信（比值 %.2f ≤3）：Phase 1 可直接用 '
              'Σ_vis,k=(JᵀJ)⁻¹ 加权。' % ratio)
    if n >= 30:
        cg = [r['cov'] for r in rec]
        print('  内点空间覆盖率(4x4,≥2) med=%.2f、p10=%.2f ⇒ 门控应加**覆盖率**项'
              '（评审第4条），不能只看内点率。' % (np.median(cg),
                                                np.percentile(cg, 10)))
    print('总耗时 %.0f s' % (time.time() - t0))


if __name__ == '__main__':
    main()
