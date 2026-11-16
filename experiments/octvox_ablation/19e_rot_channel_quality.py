#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P0-C 的决定性一测：图像旋转通道**自身**能不能用？

【口径已在 §36 用实测钉死，本脚本按新口径重跑】
 1) 外参侧别：V.R_CL 是 **b→c**（重力检验 + 侧别网格 + 3-DOF 自由拟合三方一致）。
    body 相对旋转到相机系的共轭是 conj_B(L) = R_CL · L · R_CLᵗ（下面 RCM=V.R_CL）。
    旧版本把 RCM 写成 R_CLᵗ、把 cam_rel_from_lio 写成 R_wb·R_CL，两处已订正；
    第一版 19e 的 **B 类（对 IMU/对 LIO）**数字全建立在这个错误共轭 + 错误相位 +
    IMU 左乘上，**作废、不得引用**；A 类（逆一致性/三角闭合）本来就不碰外参，
    因此 A 类是本轮唯一可信的判据，也是重跑要复核的第一件事。
 2) 时间语义（probe_time 实测）：LiDAR 周期 99.8 ms、**扫描时长也≈99.8 ms**
    ⇒ t_end[k]≈t0[k+1]，"odom 戳是头戳还是尾戳"在戳上根本不可分辨（同一 100 ms 网格），
    过去争"半周期相位"是伪问题；图像戳 = LiDAR 头戳（img[k]−t0[k]=0.0000 s, std=0）。
 3) Δt_camera（probe_ext3 实测，n=599 图像对）：B 侧残差在 **110 ms** 处出现内部极小，
    且 LIO 参照（0.387°）与 IMU 参照（0.355°）独立同点 ⇒ 图像内容比其戳晚约一帧+10 ms。
    同一 Δt 下惯性自洽地板 ∠(LIO,IMU)=0.131°，所以 0.387° 是地板的 **3.0 倍**。
    ⇒ DT0 取 0.110，扫描网格扩到 0~300 ms（只扫到 100 ms 会把"内部极小"误读成单调）。
 4) IMU 积分方向：陀螺在 body 系 ⇒ Ṙ = R[ω]× ⇒ **右乘** R(k+1)=R(k)·Exp(ω̄Δt)。
    旧版左乘把 IMU 参照污染了 0.454°。
 5) 3-DOF 外参自由拟合的结论：6 个起点（含 D=I、随机）全部收敛到 ∠(D_fit,R_CL)=0.66°、
    残差 med 0.386°（vs 固定 R_CL 的 0.387°）⇒ 换侧别后**外参没有再可调的余量**，
    剩下的误差不是口径问题。其相机系残差分量均值 [-0.001,+0.222,+0.020]°、
    标准差 [0.111,0.300,0.118]° ⇒ 0.387° 里约 0.22° 是沿相机 y 的**固定偏差**
    （正是 19b 预言的视差耦合偏差 ≈|t|/d̄，链式 ∝N 漂移，但可被 λ0 锚定/自标定吸收），
    随机散布约 0.11~0.30°。**本脚本第 1/2 段就是要独立复核这个拆分**：
    那两段既不依赖外参侧别、也不依赖 Δt_camera，是最硬的噪声地板。

================================ 口径分类 ================================
【A. 内部自洽】只反映**随机噪声**，系统偏差在闭环里相消：
 1 逆一致性 ∠(R̂12·R̂21)：正/反是两次独立跟踪+独立求解 ⇒ ≈√2·σ；
   而视差耦合偏差 ε(t) 满足 ε(−t)=−ε(t)，帧间 R 又只有零点几度 ⇒ 偏差不进入。
   ⇒ 这是**噪声地板**，且不依赖任何外部参考、不依赖 Δt_camera、不依赖外参侧别。
 2 三角闭合 ∠(R̂12·R̂23·R̂13ᵀ)：t13≈t12+t23 ⇒ 偏差共线相消 ⇒ ≈√3·σ。
【B. 外部参考】= 噪声 + 系统偏差 + 时间相位：
 3 对 IMU ∫ω 的区间旋转（200 Hz 积分，100 ms 内精度 ~0.01°，不含 ESKF 抖动）；
 4 对 LIO slerp 的区间旋转（19d 口径，含 LIO 自身抖动 —— 这正是要修的东西）。
 Δt_camera 已知（110 ms）⇒ B 类仍在网格上扫，但**看的是极小点是否复现 110 ms**，
 以此交叉验证 probe_ext3；最小值处的残差才是真实的"图像 vs 参考"偏差。

================================ 判决逻辑 ================================
 A ≲0.05° 且 B_min ≳0.3°  ⇒ 图像通道噪声够用，B 的残差几乎全是**系统性视差耦合
   偏差** ⇒ 下一步换 **LiDAR 逐点深度**（有救）。
 A ≳0.3°                  ⇒ 重复性本身就到不了 0.1° 级，任何深度源都救不了
   ⇒ "旋转优先"路线降级。
 链式漂移 ∝ N（而非 √N）   ⇒ 偏差符号固定，滑窗优化只能摊平不能消除。
 P0-C 判据（v2）：≤0.10° 主路线成立 / 0.10~0.30° 条件通过 / >0.30° 降级。
   ⇒ 必须用**随机分量**（A 类）判，而不是把 0.22° 定偏差也算进噪声里。
==========================================================================

取样说明：bag 前 ~8 s 与后 ~5 s 静止（|dp|/step 0.0025 m、gyr 0.005 rad/s），
且 TUM 轨迹比 LiDAR 头戳晚 0.796 s 才开始 ⇒ 有效窗口 k∈[90,1290]，默认 K0=100。
"""
import sys
import time

import numpy as np

sys.path.insert(0, '/home/ywj/git/Super-LIVO/experiments/octvox_ablation')
import _p0_bag as B
import _p0_visual as V
import _p0_rot as RT

NMAX = int(sys.argv[1]) if len(sys.argv) > 1 else 200
K0 = int(sys.argv[2]) if len(sys.argv) > 2 else 100
BASE = '/home/ywj/slam_data/retail_abl/base.txt'
DTG = np.round(np.arange(0.00, 0.301, 0.01), 3)   # Δt_camera 扫描网格（秒）；须覆盖 110 ms
DT0 = 0.110                                       # 深度辅助取 t_c 时用的相位（probe_ext3 实测）
MINBASE = 0.008
MODELS = ('pure', 'deb')
NAME = dict(pure='裸 Procrustes', deb='深度辅助(t=LIO)')


def b2c(L):
    """body 系区间相对旋转 -> 相机系：conj_B(L) = R_CL · L · R_CLᵗ（V.R_CL 实测是 b→c）。

    统一走这个函数，别再手写转置位置（上一版就是手写写反了侧别）。
    """
    return V.rotm(V.R_CL @ L @ V.R_CL.T)


def imu_traj(ti, gi):
    """IMU 角速度梯形积分 -> 只有旋转的 Traj（平移置零）。

    陀螺 gi 在 body 系 ⇒ Ṙ = R·[ω]×（R 把 body 向量映到起始参考系）
    ⇒ 离散递推必须**右乘**： R(k+1) = R(k)·Exp(ω̄Δt)。左乘是错的（旧版污染 0.454°）。
    """
    Rs = np.empty((len(ti), 3, 3))
    Rs[0] = np.eye(3)
    dt = np.diff(ti)
    for k in range(len(dt)):
        Rs[k + 1] = Rs[k] @ V.exp_so3(0.5 * (gi[k] + gi[k + 1]) * dt[k])
    return V.Traj(ti, np.zeros((len(ti), 3)),
                  np.array([V.rot_to_quat_w(r) for r in Rs]))


def rel(tr, t1, t2):
    """tr 的 t1->t2 区间相对旋转；端点落在空隙外返回 None。"""
    _, R1, v1 = tr.at([t1])
    _, R2, v2 = tr.at([t2])
    return None if not (v1[0] and v2[0]) else R2[0].T @ R1[0]


def med(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.median(x)) if len(x) else np.nan


def st(x, lbl):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if not len(x):
        print('  %-18s 无样本' % lbl)
        return np.nan
    print('  %-18s n=%4d  med=%7.3f°  p75=%7.3f  p90=%7.3f  max=%7.3f'
          % (lbl, len(x), np.median(x), np.percentile(x, 75),
             np.percentile(x, 90), x.max()))
    return float(np.median(x))


def main():
    t0 = time.time()
    tl, _, _ = B.read_lidar_stamps(stride=1)
    ti, gi, _ = B.read_imu()
    tum = np.loadtxt(BASE)
    lio = V.Traj(tum[:, 0], tum[:, 1:4], tum[:, 4:8])
    imu = imu_traj(ti, gi)
    ks = list(range(K0, min(K0 + NMAX, len(tl) - 3)))
    print('LiDAR %.1f ms/帧，IMU %.0f Hz，TUM %d 行（比 LiDAR 头戳晚 %.3f s）'
          % (1000 * med(np.diff(tl)), len(ti) / (ti[-1] - ti[0]), len(tum),
             tum[0, 0] - tl[0]))

    need = sorted(set(k for x in ks for k in (x, x + 1, x + 2)))
    frames = B.read_images(need)
    gray = {k: V.prep_gray(frames[k][1]) for k in frames}
    del frames
    print('取样 k=%d..%d（%.0f s）；读图 %.0f s'
          % (ks[0], ks[-1], tl[ks[-1]] - tl[ks[0]], time.time() - t0))

    # ---------------- 前端：每个有序对只跟踪一次，两模型共用同一批对应点 ----------------
    pairs = sorted(set([(k, k + 1) for k in ks] + [(k + 1, k + 2) for k in ks]
                       + [(k, k + 2) for k in ks] + [(k + 1, k) for k in ks]))
    tk = time.time()
    TR = {}
    for (i, j) in pairs:
        a, b, ntr = V.track_pair(gray[i], gray[j])
        if a is None or len(a) < 40:
            TR[(i, j)] = None
            continue
        _, tc, tn = RT.cam_rel_from_lio(lio, tl[i] + DT0, tl[j] + DT0)
        TR[(i, j)] = dict(a=a, b=b, tc=tc, tn=tn, n=ntr)
    print('跟踪 %d 有序对 %.0f s' % (len(TR), time.time() - tk))

    tk = time.time()
    EST, diag = {}, {m: dict(tn=[], px=[], sig=[], th=[], dmed=[], nv=[]) for m in MODELS}
    for key, f in TR.items():
        if f is None:
            EST[key] = dict.fromkeys(MODELS)
            continue
        dp = RT.estimate_rotation(f['a'], f['b'])
        dd = RT.estimate_rotation_d(f['a'], f['b'], f['tc']) if f['tn'] >= MINBASE \
            else dict(R=None, usable=False)
        EST[key] = dict(pure=dp, deb=dd)
        if key[1] != key[0] + 1:
            continue
        for m, d in (('pure', dp), ('deb', dd)):
            if d is None or d.get('R') is None:
                continue
            s = diag[m]
            s['tn'].append(f['tn'])
            s['th'].append(d.get('th_deg', np.nan))
            s['px'].append(d.get('px_med', np.nan))
            s['sig'].append(d.get('sigma_max_deg', np.nan))
            s['nv'].append(d.get('n_inl', np.nan))
            if m == 'deb':
                s['dmed'].append(d.get('d_med', np.nan))
    print('求解 %d 有序对×2 模型 %.0f s' % (len(EST), time.time() - tk))

    def Rof(key, m):
        e = EST.get(key)
        return None if (e is None or e.get(m) is None) else e[m].get('R')

    # ---------------- 1/2：内部自洽 = 噪声地板（与 Δt_camera 无关）----------------
    A = {m: dict(inv=[], clo=[]) for m in MODELS}
    for k in ks:
        for m in MODELS:
            r12, r21 = Rof((k, k + 1), m), Rof((k + 1, k), m)
            if r12 is not None and r21 is not None:
                A[m]['inv'].append(V.angle_of(r12 @ r21))
            r23, r13 = Rof((k + 1, k + 2), m), Rof((k, k + 2), m)
            if r12 is not None and r23 is not None and r13 is not None:
                A[m]['clo'].append(V.angle_of(r12 @ r23 @ r13.T))
    print('\n===== 1. 逆一致性 ∠(R12·R21) ≈ √2·σ_随机（噪声地板）=====')
    m_inv = {m: st(A[m]['inv'], NAME[m]) for m in MODELS}
    print('===== 2. 三角闭合 ∠(R12·R23·R13ᵀ) ≈ √3·σ_随机（零外部参考）=====')
    m_clo = {m: st(A[m]['clo'], NAME[m]) for m in MODELS}

    # ---------------- 3：Δt_camera 扫描（对 IMU / 对 LIO）----------------
    print('\n===== 3. Δt_camera 扫描：med ∠(R̂12ᵀ·R_ref) 对相位 =====')
    print('  Δt(ms)   IMU-pure  IMU-deb    LIO-pure  LIO-deb   LIO↔IMU')
    RIMU = {d: [] for d in DTG}
    RLIO = {d: [] for d in DTG}
    for d in DTG:
        for k in ks:
            RIMU[d].append(rel(imu, tl[k] + d, tl[k + 1] + d))
            RLIO[d].append(rel(lio, tl[k] + d, tl[k + 1] + d))
    sw = {m: dict(imu=[], lio=[]) for m in MODELS}
    swref = []
    for d in DTG:
        e = {m: dict(imu=[], lio=[]) for m in MODELS}
        rf = []
        for i, k in enumerate(ks):
            Ri, Rl = RIMU[d][i], RLIO[d][i]
            if Ri is not None and Rl is not None:
                rf.append(V.angle_of(b2c(Rl @ Ri.T)))
            for m in MODELS:
                r = Rof((k, k + 1), m)
                if r is None:
                    continue
                if Ri is not None:
                    e[m]['imu'].append(V.angle_of(r.T @ b2c(Ri)))
                if Rl is not None:
                    e[m]['lio'].append(V.angle_of(r.T @ b2c(Rl)))
        for m in MODELS:
            sw[m]['imu'].append(med(e[m]['imu']))
            sw[m]['lio'].append(med(e[m]['lio']))
        swref.append(med(rf))
        print('  %5.0f   %8.3f  %8.3f   %9.3f  %8.3f  %8.3f'
              % (1000 * d, sw['pure']['imu'][-1], sw['deb']['imu'][-1],
                 sw['pure']['lio'][-1], sw['deb']['lio'][-1], swref[-1]))
    JDM = 'deb'                                  # 要上车的估计器：深度辅助
    jbest = {m: int(np.nanargmin(np.asarray(sw[m]['imu']))) for m in MODELS}
    jlio = {m: int(np.nanargmin(np.asarray(sw[m]['lio']))) for m in MODELS}
    DTB = float(DTG[jbest[JDM]])
    for m in MODELS:
        print('  ⇒ [%s] 最优相位：vs IMU %.0f ms（%.3f°）  vs LIO %.0f ms（%.3f°）'
              % (NAME[m], 1000 * DTG[jbest[m]], sw[m]['imu'][jbest[m]],
                 1000 * DTG[jlio[m]], sw[m]['lio'][jlio[m]]))
    print('  ⇒ 复核：主判据模型（%s）在 IMU / LIO 两个参照上的极小点分别落在 '
          '%.0f / %.0f ms%s ⇒ probe_ext3 的 110 ms %s'
          % (NAME[JDM], 1000 * DTG[jbest[JDM]], 1000 * DTG[jlio[JDM]],
             '（同点）' if jbest[JDM] == jlio[JDM] else '（不同点，需回查）',
             '复现' if abs(DTG[jbest[JDM]] - DT0) <= 0.02 else
             '未复现（实测 %.0f ms，须以本脚本为准）' % (1000 * DTG[jbest[JDM]])))
    print('    裸 3-DOF 的极小点在 %.0f ms：它没有平移模型，视差耦合偏差把极小点从真相位'
          '推开了 ⇒ **不可用裸 Procrustes 的相位判定 Δt_camera**'
          % (1000 * DTG[jbest['pure']]))
    # ---------------- 4：参考之间的一致性 + 随机/定偏差拆分 ----------------
    print('\n===== 4. 残差的构成拆分（Δt=%.0f ms，主判据模型 %s）====='
          % (1000 * DTB, NAME[JDM]))
    rf = []
    for i, k in enumerate(ks):
        Ri, Rl = RIMU[DTB][i], RLIO[DTB][i]
        if Ri is not None and Rl is not None:
            rf.append(V.angle_of(b2c(Rl) @ b2c(Ri).T))
    FLOOR = st(rf, 'LIO ↔ IMU')
    print('  （此值 = LIO 与 IMU 两者区间误差的合成；若 >> 图像残差 ⇒ 参照本身在抖）')

    # 把「图像 vs IMU」的残差向量拆成 定偏差(均值) + 随机(去均值后的散布)
    print('\n  ---- 4b. 图像 vs IMU 残差 Log(R̂ᵗ·R_ref) 的 定偏差 / 随机 拆分 ----')
    BIAS = {}
    for m in MODELS:
        rv = []
        for i, k in enumerate(ks):
            r = Rof((k, k + 1), m)
            Ri = RIMU[DTB][i]
            if r is not None and Ri is not None:
                rv.append(V.log_so3(r.T @ b2c(Ri)))
        if len(rv) < 30:
            print('  [%s] 样本不足(%d)' % (NAME[m], len(rv)))
            continue
        # log_so3 返回**弧度**，本段全部要换成度（angle_of 才自带度）
        E = np.degrees(np.array(rv))
        mu = E.mean(axis=0)
        sd = E.std(axis=0)
        resid = E - mu
        # 总残差模长 = √(|μ|² + 随机²) 只有在两者正交时才严格成立，故同时给直测量
        BIAS[m] = dict(n=len(E), mu=mu, sd=sd, nm=float(np.linalg.norm(mu)),
                       rnd=float(np.median(np.linalg.norm(resid, axis=1))),
                       tot=float(np.median(np.linalg.norm(E, axis=1))))
        print('  [%s] n=%d  总 med=%.3f°' % (NAME[m], len(E), BIAS[m]['tot']))
        print('      定偏差 μ=[%+.3f, %+.3f, %+.3f]°  |μ|=%.3f°  (相机 x,y,z)'
              % (mu[0], mu[1], mu[2], BIAS[m]['nm']))
        print('      随机轴向 σ=[%.3f, %.3f, %.3f]°  去均值后模长 med=%.3f°'
              % (sd[0], sd[1], sd[2], BIAS[m]['rnd']))
        print('      ⇒ 定偏差占总残差 %.0f%%（若能标定/被 λ0 锚定吸收，上车看随机分量）'
              % (100 * BIAS[m]['nm'] / max(BIAS[m]['tot'], 1e-9)))

    # ---------------- 5：链式漂移（在最优相位）----------------
    print('\n===== 5. 链式漂移 ∠(∏_{j=k}^{k+N-1} R̂(j→j+1) · R_IMU(k→k+N)ᵀ) =====')
    print('   N     n    pure med    deb med   |  √N 外推(纯噪声)   N 线性(定符号偏差)')
    CH = {}
    for N in (2, 3, 5, 10, 15, 20):
        ep, ed = [], []
        for k in ks[:len(ks) - N]:
            keys = [(j, j + 1) for j in range(k, k + N)]
            if any(TR.get(q) is None or TR[q]['tn'] < MINBASE for q in keys):
                continue
            accs = {m: np.eye(3) for m in MODELS}
            bad = False
            for q in keys:
                for m in MODELS:
                    r = Rof(q, m)
                    if r is None:
                        bad = True
                        break
                    accs[m] = r @ accs[m]
                if bad:
                    break
            if bad:
                continue
            Ri = rel(imu, tl[k] + DTB, tl[k + N] + DTB)
            if Ri is None:
                continue
            Ri = b2c(Ri)
            ep.append(V.angle_of(accs['pure'].T @ Ri))
            ed.append(V.angle_of(accs['deb'].T @ Ri))
        CH[N] = (med(ep), med(ed))
        if ep:
            print('  %3d %4d   %9.3f  %9.3f   |  %9.3f            %9.3f'
                  % (N, len(ep), CH[N][0], CH[N][1],
                     CH[2][0] * np.sqrt(N / 2.0), CH[2][0] * N / 2.0))
        else:
            print('  %3d    0  ——' % N)

    # ---------------- 6：诊断 ----------------
    print('\n===== 6. 相邻对诊断（解释上面这些数）=====')
    for m in MODELS:
        d = diag[m]
        if not d['tn']:
            print('  [%s] 无有效样本' % NAME[m])
            continue
        extra = '' if m == 'pure' else '  d̄ med=%6.1f m  |  σ̂ med=%.4f°' \
            % (med(d['dmed']), med(d['sig']))
        print('  [%s] n=%3d/%d  |R̂| med=%.3f°  |t| med=%.4f m  n_inl=%4d  '
              'px_med=%.3f%s' % (NAME[m], len(d['tn']), len(ks), med(d['th']),
                                 med(d['tn']), int(med(d['nv'])), med(d['px']), extra))

    # ---------------- 7：判决 ----------------
    print('\n===== 7. 判决（v2 三档：≤0.10 主路线 / 0.10~0.30 条件通过 / >0.30 降级）=====')
    a_noise = min([v for v in m_clo.values() if np.isfinite(v)] +
                  [v for v in m_inv.values() if np.isfinite(v)] + [np.inf])
    b_imu = sw[JDM]['imu'][jbest[JDM]]
    b_lio = sw[JDM]['lio'][jbest[JDM]]
    bd = BIAS.get(JDM)
    # 「随机分量」才是 P0-C 三档判据要量的东西：
    #   优先用 4b 的去均值散布（含参照误差），并与 A 类地板取更可信者。
    rnd = bd['rnd'] if bd else a_noise
    print('  A 噪声地板（内部自洽，不碰外参/不碰 Δt） med = %.3f°' % a_noise)
    for m in MODELS:
        if np.isfinite(m_inv[m]):
            print('     [%s] 逆一致性 %.3f° ⇒ σ≈%.3f°(÷√2)   三角闭合 %.3f° ⇒ σ≈%.3f°(÷√3)'
                  % (NAME[m], m_inv[m], m_inv[m] / np.sqrt(2),
                     m_clo[m], m_clo[m] / np.sqrt(3)))
    print('  B 图像 vs IMU @最优相位     med = %.3f°  (Δt=%.0f ms)  [主判据模型 %s]'
          % (b_imu, 1000 * DTB, NAME[JDM]))
    print('  B 图像 vs LIO @最优相位     med = %.3f°' % b_lio)
    print('  参考之间 LIO↔IMU 地板       med = %.3f°' % FLOOR)
    if bd:
        print('  拆分：定偏差 |μ|=%.3f° + 随机 med=%.3f°（总 %.3f°）' %
              (bd['nm'], bd['rnd'], bd['tot']))
        print('  三档判据取**随机分量** %.3f°（定偏差可标定/被锚定吸收，不计入噪声）' % rnd)
    print('  纯平滑口径失效提示：§36 已实测 hf_θ(LIO)=0.519° 里 95%% 是真实角运动曲率，'
          '故本判决的对齐目标是「区间旋转误差」≈%.2f°，不是 hf_θ≤0.20°。'
          % (np.hypot(FLOOR, rnd)))
    if rnd > 0.30:
        print('  ⇒ 降级：图像两帧旋转的**重复性** %.3f° 已超 0.30° 档，'
              '随机噪声本身就够不到 0.1° 目标，换深度源救不了。' % rnd)
    elif b_imu - FLOOR > 0.30:
        print('  ⇒ 随机 %.3f° 够低，但对参照仍有 %.3f° ⇒ **系统性偏差主导**（非噪声）。'
              '下一步：LiDAR 逐点深度重测同一口径；偏差若可标定，则 λ0 锚定/自标定可吸收。'
              % (rnd, b_imu))
    elif rnd > 0.10:
        print('  ⇒ 条件通过区：随机 %.3f° ∈(0.10,0.30]，B=%.3f°。'
              '需由 P0-D 的重复估计散布 + 地图收益决定是否放行，不得直接开 Phase 1。'
              % (rnd, b_imu))
    else:
        print('  ⇒ 图像通道可用（随机 %.3f° / 定偏差 %.3f°），旋转优先路线成立；'
              '但 0.1° 级还需 P0-D 的重复估计散布佐证。'
              % (rnd, bd['nm'] if bd else np.nan))
    if 5 in CH and 20 in CH and np.isfinite(CH[5][1]) and CH[5][1] > 0:
        r = CH[20][1] / CH[5][1]
        print('  链式标度 %s: N=5→20 比值 %.2f（√N⇒1.41，定符号偏差⇒4.0）' % (NAME[JDM], r))
    print('总耗时 %.0f s' % (time.time() - t0))


if __name__ == '__main__':
    main()
