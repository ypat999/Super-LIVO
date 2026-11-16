#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P0-C 真实数据：深度辅助旋转（t 来自 LIO）能否把轴偏差从 1.293° 压到 ≤0.15°？

⚠【§36：本脚本的残差绝对值不得引用，结论由 19e 取代】
本脚本的参照旋转 e 在**错误的共轭侧别**下计算（V.R_CL 实测是 b→c，共轭应为
R_CL·L·R_CLᵗ；此处写成 R_CLᵗ·L·R_CL），且 Δt_camera 仍按旧假设扫 ±100 ms
（实测 = 110 ms，超出旧网格）。⇒ 它给出的"深度辅助后仍 ~0.3°"这一档偏悲观，
新口径实测：深度辅助 vs IMU **0.048°**（随机 0.043° + 定偏差 0.018°）。
本脚本**定性结论仍有效**：裸 3-DOF 与 LIO 的轴对不上主要是视差耦合 + 估计器退化，
换深度辅助（t 已知）能大幅压下来 —— 这一点由 19e/19f/19g 独立复核。
用法参数与窗口也请改用 19e：`python3 19e_rot_channel_quality.py 500 100`。

19a 实测：裸两帧纯旋转与 LIO 的旋转**幅值**一致（|r_vis|/|r_lio| med 1.061）但**轴**对不上，
拟合最优固定正交 M 后仍 med=1.293° ⇒ 不是外参、不是时间同步。
19b/19c 定性定量解释该残差 = 视差耦合，量级 ≈ |t|/d̄；合成真值下"t 已知 + 单主导平面"
的 6 参数模型把它压到 0.02~0.05°。本脚本在 bag 上验同一件事。

两个残差口径都给（都很重要）：
  ∠(R_visᵀ R_LIO)   —— 这就是第一阶段因子图里 r_vis 的实际取值 ⇒ 判决用它
  拟合 M 后 ‖log R_vis − M log R_LIO‖ —— 与 19a 的 1.293° 同定义，便于直接比较
初值口径也给两种：
  init=LIO   ：符合真实架构（refiner 以 LIO 为锚做 basin 选择），但会把答案喂给初值
  init=none  ：完全独立的视觉解（裸 Procrustes 交替起步），更严格
⇒ 只有 **init=none 也达标**才算旋转通道真的可用。

同时顺带把 18_ 的 C/D 段用可用估计器重跑：扫曝光中点偏置 Δ，轴残差最小处即 Δt_camera
（旧的 C/D 用坏掉的 5-point R，结论作废；A/B 段结论不变）。

用法：python3 19d_rot_debias_real.py [pair数=160] [步长=8]
"""
import sys
import time

import numpy as np

sys.path.insert(0, '/home/ywj/git/Super-LIVO/experiments/octvox_ablation')
import _p0_bag as B
import _p0_visual as V
import _p0_rot as RT

NPAIR = int(sys.argv[1]) if len(sys.argv) > 1 else 160
STEP = int(sys.argv[2]) if len(sys.argv) > 2 else 8
BASE = '/home/ywj/slam_data/retail_abl/base.txt'
DGRID = np.arange(-0.10, 0.101, 0.02)
MINROT = 0.25          # deg，旋转太小则轴方向是纯噪声


def orth_fit(P, Q):
    """正交 M 使 Q ≈ M·P。"""
    C = Q.T @ P
    U, _, Vt = np.linalg.svd(C)
    return U @ np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))]) @ Vt


def main():
    t0 = time.time()
    tl, _, _ = B.read_lidar_stamps(stride=1)
    tum = np.loadtxt(BASE)
    lio = V.Traj(tum[:, 0], tum[:, 1:4], tum[:, 4:8])
    dt_iv = float(np.median(np.diff(tl)))
    print('LiDAR 帧周期 med=%.2f ms；odom dt med=%.2f ms；base 行数=%d'
          % (dt_iv * 1e3, np.median(np.diff(tum[:, 0])) * 1e3, len(tum)))

    idx = [k for k in range(0, len(tl) - STEP, STEP)][:NPAIR]
    idx = [k for k in idx if 0.05 < tl[k + 1] - tl[k] < 0.15]
    print('取样 %d 对（步长 %d，覆盖 %.0f s）' % (len(idx), STEP, tl[idx[-1]] - tl[idx[0]]))
    want = sorted(set(idx) | set(k + 1 for k in idx))
    tk = time.time()
    frames = B.read_images(want)
    gray = {k: V.prep_gray(frames[k][1]) for k in want}
    print('读图+去畸变 %d 张 %.1f s' % (len(want), time.time() - tk))

    tk = time.time()
    prs = []
    for k in idx:
        a, b, ntr = V.track_pair(gray[k], gray[k + 1])
        if a is not None and len(a) >= 40:
            prs.append((k, a, b))
    print('跟踪有效 %d 对，%.1f s（%.0f ms/对）'
          % (len(prs), time.time() - tk, (time.time() - tk) / max(1, len(idx)) * 1e3))

    def sweep(mode):
        """mode: 'pure' 裸 Procrustes | 'debias_none' | 'debias_lio'。
        -> {Δ: dict(fac=∠(RvᵀRl), rv=, rl=, d=估计器输出列表)}"""
        out = {}
        for d in DGRID:
            fac, rv, rl, dl = [], [], [], []
            for k, a, b in prs:
                Rl, tc, tn = RT.cam_rel_from_lio(lio, tl[k] + d, tl[k] + d + dt_iv)
                if Rl is None or abs(V.angle_of(Rl)) < MINROT:
                    continue
                if mode == 'pure':
                    dd = RT.estimate_rotation(a, b)
                elif tn >= 0.008:
                    dd = RT.estimate_rotation_d(a, b, tc,
                                                R0=(Rl if mode == 'debias_lio' else None))
                else:
                    continue
                Rv = dd.get('R')
                if Rv is None:
                    continue
                fac.append(V.angle_of(Rv.T @ Rl))
                rv.append(V.log_so3(Rv))
                rl.append(V.log_so3(Rl))
                dl.append(dd)
            out[d] = dict(fac=np.array(fac), rv=np.array(rv), rl=np.array(rl), d=dl)
        return out

    def show(lbl, sw, with_det=True):
        print('\n===== %s =====' % lbl)
        if with_det:
            print(' Δ(ms)    n   med∠fac   p75    p90   |t|med   d̄med  px_med  σ̂max |  M拟合残差')
        best, bv, bx = None, 1e9, None
        for d in DGRID:
            o = sw[d]
            n = len(o['fac'])
            if n < 30:
                continue
            f = o['fac']
            M = orth_fit(o['rl'], o['rv'])
            emf = np.degrees(np.linalg.norm(o['rv'] - (M @ o['rl'].T).T, axis=1))
            row = ' %+5.0f  %4d  %7.3f  %6.3f %6.3f' % (
                d * 1e3, n, np.median(f), np.percentile(f, 75), np.percentile(f, 90))
            if with_det:
                dd = o['d']
                row += '  %6.4f  %5.1f %6.3f %6.4f' % (
                    np.median([x['t_norm'] for x in dd]), np.median([x['d_med'] for x in dd]),
                    np.median([x['px_med'] for x in dd]), np.median([x['sigma_max_deg'] for x in dd]))
            row += ' |  %6.3f' % np.median(emf)
            print(row)
            if np.median(f) < bv:
                best, bv, bx = d, np.median(f), o
        if best is None:
            print(' → 无有效样本（n<30），该口径作废')
            return None, np.nan, None
        mfbx = np.degrees(np.linalg.norm(
            bx['rv'] - (orth_fit(bx['rl'], bx['rv']) @ bx['rl'].T).T, axis=1))
        print(' → 最优 Δ=%+.0f ms：∠fac med=%.3f° p90=%.3f°；M 拟合后 med=%.3f°  (n=%d)'
              % (best * 1e3, bv, np.percentile(bx['fac'], 90), np.median(mfbx),
                 len(bx['fac'])))
        return best, bv, bx

    r_pure = sweep('pure')
    show('A. 裸两帧纯旋转（对照，19a 的 M 残差≈1.293° 应在这一行复现）', r_pure, with_det=False)
    r_dn = sweep('debias_none')
    d_dn, v_dn, o_dn = show('B. 深度辅助，初值=裸 Procrustes（完全独立的视觉解）', r_dn)
    r_dl = sweep('debias_lio')
    d_dl, v_dl, o_dl = show('C. 深度辅助，初值=LIO（真实架构口径）', r_dl)

    print('\n===== D. σ̂ 标定（LM 处 JᵀJ 边缘协方差 vs 实测散布）=====')
    if o_dn is not None:
        sg = np.array([x['sigma_max_deg'] for x in o_dn['d']])
        rs = np.array(o_dn['fac'])
        for lo, hi in ((0.0, 0.5), (0.5, 1.5), (1.5, 3.0), (3.0, 99.)):
            m = (rs >= lo) & (rs < hi)
            if m.sum() >= 8:
                print('  ∠fac∈[%4.1f,%4.1f)°: n=%3d  σ̂ med=%7.4f°  实测 med=%6.3f°  比=%5.1f×'
                      % (lo, hi, m.sum(), np.median(sg[m]), np.median(rs[m]),
                         np.median(rs[m]) / max(1e-9, np.median(sg[m]))))

    print('\n===== E. 判决 =====')
    print('判据（v2 P0-C）：∠fac med ≤0.15° 可用 / 0.15–0.30° 条件通过 / >0.30° 需 LiDAR 逐点深度')

    def verdict(v):
        if not np.isfinite(v):
            return '无样本'
        return '可用' if v <= 0.15 else ('条件通过' if v <= 0.30 else '不通过')

    vp = np.median(r_pure[d_dn]['fac']) if (d_dn is not None and len(r_pure[d_dn]['fac'])) \
        else np.nan
    for lbl, v in (('裸 Procrustes', vp), ('深度辅助(init=none)', v_dn),
                   ('深度辅助(init=LIO)', v_dl)):
        print('  %-22s med=%7.3f°  ⇒ %s' % (lbl, v, verdict(v)))
    print('Δt_camera（旋转通道独立测量）= %s ms（init=none）/ %s ms（init=LIO）'
          % ('%+.0f' % (d_dn * 1e3) if d_dn is not None else 'n/a',
             '%+.0f' % (d_dl * 1e3) if d_dl is not None else 'n/a'))
    print('总耗时 %.0f s' % (time.time() - t0))


if __name__ == '__main__':
    main()
