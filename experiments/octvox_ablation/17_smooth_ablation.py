#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P0-B 离线判决实验：纯位姿平滑（完全不引入图像信息）能做到什么。

Super-LIO 每帧发布独立 ESKF 后验，帧间位姿有微量高频抖动 -> 彩色点云地图分层。
在决定是否上「图像位姿修正器」之前，必须先量化「纯平滑」这条下限。

主输入  /home/ywj/slam_data/retail_abl/base.txt   TUM: t x y z qx qy qz qw（标量在后）

三类平滑器（口径严格分开，不得横向比较）
  A 离线零相位      ：filtfilt / 中心窗口 SG —— 只是「理论最好能压到多少」的参照上限，不可部署
  B 在线因果        ：EMA 一阶低通 / 二阶前向差分正则最小方差 —— 真正的候选
  C 允许固定延迟滑窗：窗口内最小二乘多项式拟合，输出窗口最老一帧，延迟 (W-1)xdt

每个操作点同时输出四类指标：
  (1) hf_p / hf_θ 的 P50/P90（同一 3-tap 口径）
  (2) 相对原始 LIO 的偏移（姿态角差 / 位置差）
  (3) 真实运动误差（快速转向段的最佳时间偏移 = 滞后量，及 5 m 深度等效切向位移；GT 上的 ATE）
  (4) 算法延迟

跑法：python3 17_smooth_ablation.py   （无参数，结果全部打印到 stdout）
"""

import json
import os
import sys

import numpy as np
from scipy import signal as sps
from scipy.spatial.transform import Rotation as RO

# ----------------------------------------------------------------------------
# 常量
# ----------------------------------------------------------------------------
BASE = '/home/ywj/slam_data/retail_abl/base.txt'
GAP_MAX = 0.30          # s：3-tap 跨越这个空隙的帧作废
DEPTH = 5.0             # m：等效切向位移换算深度
TURN_TH = 20.0          # deg/s：快速转向判据
NOMINAL_DT = 0.10       # s：10 Hz 标称帧周期
THETA_JUDGE = 0.15      # deg：判决阈值（hf_θ P50）
EQ_JUDGE = 5.0          # mm：判决阈值（转向段滞后等效位移）

NTU_BAG = '/home/ywj/slam_data/ntu_viral/run_ntu_off'
NTU_GT = '/home/ywj/slam_data/ntu_viral/gt/eee_03/ground_truth.csv'
RTK_ROOT = '/home/ywj/slam_data/rtk_slam/eval_ref'
RTK_TRAJ = RTK_ROOT + '/trajectories/super_lio/construction_seq2/traj_offline.txt'
RTK_GTCSV = RTK_ROOT + '/ground_truth/construction_seq2.csv'
RTK_ORIGIN = RTK_ROOT + '/trajectories/super_lio/construction_seq2/enu_origin.json'

# 上一轮已实测确认的锚点（mm / mdeg）
ANCHOR = dict(hp_p50=3.85, hp_p90=6.71, hp_max=43.3, ht_p50=510.0, ht_p90=998.0)

W = 132                 # 输出宽度


# ----------------------------------------------------------------------------
# 几何工具
# ----------------------------------------------------------------------------
def load_tum(path):
    """TUM -> (t, p, rot)。四元数按 [x,y,z,w] 读入（正是 scipy 的约定）。"""
    a = np.loadtxt(path)
    q = a[:, 4:8]
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    return a[:, 0], a[:, 1:4], RO(q)


def valid_mask(t):
    """对 k=1..N-2：两侧相邻间隔都 <= GAP_MAX 时 3-tap 才有效。"""
    return (t[1:-1] - t[:-2] <= GAP_MAX) & (t[2:] - t[1:-1] <= GAP_MAX)


def ang_between(r1, r2):
    """逐元素夹角 rad：|Log(R1^T R2)|。"""
    return (r1.inv() * r2).magnitude()


def geo_midpoint(r1, r2):
    """测地中点 = R1 * exp(0.5 * Log(R1^T R2)) = slerp(R1, R2, 0.5)。
       用旋转矩阵/rotvec 表达，符号 (q -> -q) 问题天然不存在。"""
    return r1 * RO.from_rotvec(0.5 * (r1.inv() * r2).as_rotvec())


def increments(rot):
    """世界系（左平凡化）角位移 rotvec 序列，长度 N-1：W_k = Log(R_{k+1} R_k^T)。
       不用体帧（右平凡化）增量：那样滤波误差会经由「在已偏离的姿态上继续叠加增量」
       被旋转放大，重投影反馈可把偏差推到上百 degrees。左平凡化下
       R_k = exp(W_{k-1})...exp(W_0) R_0 精确成立，增量方向固定在世界系，无反馈。"""
    return (rot[1:] * rot[:-1].inv()).as_rotvec()


def cum_rot(r0, dvec):
    """由世界系增量累积重建轨迹：R_{k+1} = exp(W_k) R_k。
       滤波（LTI）与累加可交换，故这等价于「对绝对 rotation vector 滤波」（差 BCH 二阶项）。"""
    Wd = RO.from_rotvec(dvec).as_matrix()
    out = np.empty((len(dvec) + 1, 3, 3))
    out[0] = r0.as_matrix()
    for i in range(len(dvec)):
        out[i + 1] = Wd[i] @ out[i]
    return RO.from_matrix(out)


def pos_interp(t, p, tq):
    i = np.clip(np.searchsorted(t, tq) - 1, 0, len(t) - 2)
    sp = np.maximum(t[i + 1] - t[i], 1e-12)
    u = np.clip((tq - t[i]) / sp, 0.0, 1.0)[:, None]
    return p[i] * (1.0 - u) + p[i + 1] * u


def rot_interp(t, rot, tq):
    """分段匀速测地插值。"""
    i = np.clip(np.searchsorted(t, tq) - 1, 0, len(t) - 2)
    sp = np.maximum(t[i + 1] - t[i], 1e-12)
    u = np.clip((tq - t[i]) / sp, 0.0, 1.0)
    d = (rot[i].inv() * rot[i + 1]).as_rotvec()
    return rot[i] * RO.from_rotvec(u[:, None] * d)


def ang_rate(t, rot, half):
    """中心差分角速度 (deg/s)，基线 2*half 帧；两端退化。half=0 -> 逐帧增量。"""
    n = len(t)
    if half == 0:
        d = np.linalg.norm(increments(rot), axis=1) / np.maximum(np.diff(t), 1e-12)
        return np.degrees(np.r_[d[0], d])
    lo = np.clip(np.arange(n) - half, 0, n - 1)
    hi = np.clip(np.arange(n) + half, 0, n - 1)
    da = ang_between(rot[lo], rot[hi])
    return np.degrees(da / np.maximum(t[hi] - t[lo], 1e-12))


# ----------------------------------------------------------------------------
# 第一步口径：3-tap 二阶差分抖动
# ----------------------------------------------------------------------------
def hf_metrics(t, p, rot):
    """3-tap 二阶差分抖动。返回 (hf_p[m], hf_θ[rad])；mdeg = degrees(hf_θ)*1e3。"""
    v = valid_mask(t)
    hp = np.linalg.norm(p[1:-1] - (p[:-2] + p[2:]) / 2, axis=1)[v]
    ht = ang_between(geo_midpoint(rot[:-2], rot[2:]), rot[1:-1])[v]
    return hp, ht


def mdeg(ht_rad):
    """弧度 -> 毫度。"""
    return np.degrees(ht_rad) * 1e3


# ----------------------------------------------------------------------------
# 在线因果滤波核
# ----------------------------------------------------------------------------
def ema_rec(y, a):
    """离散时间一阶 EMA。α 固定（不按 dt 自适应），保证 DC 增益严格为 1，
       累积重建不会注入漂移。空隙那 1 个样本按普通样本处理。"""
    out = np.empty_like(y)
    out[0] = y[0]
    for i in range(1, len(y)):
        out[i] = out[i - 1] + a * (y[i] - out[i - 1])
    return out


def minvar2_rec(z, lam):
    """min_x ||x_k - z_k||^2 + lam*||x_k - 2x_{k-1} + x_{k-2}||^2 的因果解（只用过去帧）。"""
    out = np.empty_like(z)
    out[0] = z[0]
    out[1] = (z[1] + lam * out[0]) / (1.0 + lam)          # 缺 x_{-1}，退化成零速先验
    for i in range(2, len(z)):
        out[i] = (z[i] + lam * (2.0 * out[i - 1] - out[i - 2])) / (1.0 + lam)
    return out


# ----------------------------------------------------------------------------
# 三类平滑器工厂：返回 (显示名, info, f(t,p,rot)->(p2,rot2))
# ----------------------------------------------------------------------------
def mk_butter_ff(fc, fs, order=4):
    b, a = sps.butter(order, fc / (fs / 2.0), btype='low')
    nq = float(abs(sps.freqz(b, a, worN=[np.pi])[1][0]) ** 2)      # filtfilt = H 两次

    def f(t, p, rot):
        d = increments(rot)
        pad = 3 * max(len(b), len(a)) - 1
        pn = sps.filtfilt(b, a, p, axis=0, padtype='odd', padlen=min(pad, len(p) - 1))
        dn = sps.filtfilt(b, a, d, axis=0, padtype='odd', padlen=min(pad, len(d) - 1))
        return pn, cum_rot(rot[0], dn)
    return ('A1 Butterworth-filtfilt fc=%.1fHz' % fc,
            dict(kind='offline-ff', fc=fc, nyq=nq), f)


def mk_savgol_center(win, poly):
    x = np.cos(np.pi * np.arange(6 * win))                          # Nyquist 交替序列
    nq = float(np.abs(sps.savgol_filter(x, win, poly, mode='interp')[3 * win:]).max())

    def f(t, p, rot):
        d = increments(rot)
        pn = sps.savgol_filter(p, win, poly, axis=0, mode='interp')
        dn = sps.savgol_filter(d, win, poly, axis=0, mode='interp')
        return pn, cum_rot(rot[0], dn)
    return ('A2 SavGol-center w=%d p=%d' % (win, poly),
            dict(kind='offline-center', win=win, poly=poly, nyq=nq), f)


def mk_ema(alpha):
    def f(t, p, rot):
        return ema_rec(p, alpha), cum_rot(rot[0], ema_rec(increments(rot), alpha))
    return ('B1 EMA alpha=%.4f' % alpha, dict(kind='causal-1st', alpha=alpha), f)


def mk_minvar2(lam):
    def f(t, p, rot):
        return minvar2_rec(p, lam), cum_rot(rot[0], minvar2_rec(increments(rot), lam))
    return ('B2 MinVar2 lam=%.4f' % lam, dict(kind='causal-2nd', lam=lam), f)


def mk_window_poly(win, order):
    """因果滑窗：在 [k, k+win-1] 上做 order 阶 LS 多项式拟合，输出最老一帧 k 的拟合值。
       旋转在窗口内以最老帧为参考做 log 映射（局部切空间，chart-free）。"""
    tau = np.arange(win, dtype=float)                       # 均匀 dt 时窗口权重恒定 => 内点 LTI
    Aw = np.stack([tau ** k for k in range(order + 1)], axis=-1)
    nq = float(abs(np.dot(np.linalg.solve(Aw.T @ Aw + 1e-9 * np.eye(order + 1), Aw.T)[0],
                          np.cos(np.pi * tau))))

    def f(t, p, rot):
        n = len(t)
        m = max(n - win + 1, 0)
        Rall = rot.as_matrix()
        outp = np.empty_like(p)
        outr = np.empty_like(Rall)
        if m > 0:
            tw = np.lib.stride_tricks.sliding_window_view(t, win)          # (m,win)
            pw = np.transpose(np.lib.stride_tricks.sliding_window_view(p, win, axis=0),
                              (0, 2, 1))                                    # (m,win,3)
            A = np.stack([((tw - tw[:, :1]) / NOMINAL_DT) ** k
                          for k in range(order + 1)], axis=-1)              # (m,win,order+1)
            AtA = A.transpose(0, 2, 1) @ A
            sol = np.linalg.solve(AtA + 1e-9 * np.eye(order + 1),
                                  A.transpose(0, 2, 1) @ pw)                 # (m,o+1,3)
            outp[:m] = sol[:, 0, :]
            Rt = np.transpose(Rall[:m], (0, 2, 1))
            phi = np.stack([RO.from_matrix(np.einsum('nij,njk->nik', Rt, Rall[j:j + m]))
                            .as_rotvec() for j in range(win)], axis=1)       # (m,win,3)
            sol2 = np.linalg.solve(AtA + 1e-9 * np.eye(order + 1),
                                   A.transpose(0, 2, 1) @ phi)
            outr[:m] = np.einsum('nij,njk->nik', Rall[:m],
                                 RO.from_rotvec(sol2[:, 0, :]).as_matrix())
            # 尾部不足一个完整窗口的 win-1 帧：部署时还没到，这里保留原值占位
            # （只影响最后 <=8 个样本，对 P50/P90 无影响）
            outp[m:] = p[m:]
            outr[m:] = Rall[m:]
        else:
            outp, outr = p.copy(), Rall
        return outp, RO.from_matrix(outr)
    return ('C WindowPolyLS W=%d ord=%d' % (win, order),
            dict(kind='fixed-delay', win=win, order=order, nyq=nq), f)


# ----------------------------------------------------------------------------
# 算法延迟（解析）
# ----------------------------------------------------------------------------
def iir_gd_frames(hfun, wmax=0.02):
    """低频群延迟（帧）= -darg H(e^{jw})/dw |_{w->0}。"""
    w = np.linspace(1e-5, wmax, 25)
    ph = np.unwrap(np.angle(hfun(w)))
    return float(-np.polyfit(w, ph, 1)[0])


def analytic_delay(info, dt=NOMINAL_DT):
    k = info['kind']
    if k == 'offline-ff':
        return 0.0                                     # 零相位，无前向延迟（但需要整段数据）
    if k == 'offline-center':
        return (info['win'] - 1) / 2.0 * dt * 1e3
    if k == 'causal-1st':
        a = info['alpha']
        return iir_gd_frames(lambda w: a / (1.0 - (1.0 - a) * np.exp(-1j * w))) * dt * 1e3
    if k == 'causal-2nd':
        lam = info['lam']
        return iir_gd_frames(lambda w: 1.0 / ((1.0 + lam) - 2.0 * lam * np.exp(-1j * w)
                                              + lam * np.exp(-2j * w))) * dt * 1e3
    if k == 'fixed-delay':
        return (info['win'] - 1) * dt * 1e3
    return float('nan')


def nyquist_gain(info):
    """|H(-1)|：对逐帧交替（Nyquist）抖动分量的抑制比，用来解释 hf 的下降幅度。
       3-tap 二阶差分把 Nyquist 分量放大 2 倍，滤波前后比值正好是 |H(-1)|。
       工厂已算好的直接用（含 filtfilt=H^2、SG 与滑窗的实测核响应）。"""
    if 'nyq' in info:
        return float(info['nyq'])
    k = info['kind']
    if k == 'causal-1st':
        a = info['alpha']
        return a / (2.0 - a)
    if k == 'causal-2nd':
        return 1.0 / (1.0 + 4.0 * info['lam'])
    return float('nan')


# ----------------------------------------------------------------------------
# 指标 (2) 偏移 / (3) 真实运动误差
# ----------------------------------------------------------------------------
def dev_metrics(p, rot, p2, rot2):
    da = np.degrees(ang_between(rot, rot2))
    dp = np.linalg.norm(p - p2, axis=1) * 1e3
    return dict(ang_p50=np.percentile(da, 50), ang_p95=np.percentile(da, 95),
                pos_p50=np.percentile(dp, 50), pos_p95=np.percentile(dp, 95))


def lag_metrics(t, p, rot, p2, rot2, turn, dts=None, half=1):
    """把平滑后轨迹整体平移 dt 去贴合原始轨迹，扫 dt 求最小误差。
       误差取 5 m 深度处的等效点位移 sqrt(|dp|^2 + (DEPTH*dθ)^2) 的 RMS。
       dt* > 0 表示平滑轨迹滞后 dt* 秒。half = 求角速度用的中心差分基线（帧）。"""
    if dts is None:
        dts = np.arange(-0.30, 0.30001, 0.0025)
    tk = t[turn]
    if len(tk) < 20:
        return dict(dt_ms=np.nan, res_mm=np.nan, eq_mm=np.nan, dt_rot_ms=np.nan)
    pk, rk = p[turn], rot[turn]
    wk = ang_rate(t, rot, half)[turn]
    E = np.empty(len(dts)); Ep = np.empty(len(dts)); Er = np.empty(len(dts))
    for i, dd in enumerate(dts):
        tq = tk + dd
        dp = np.linalg.norm(pos_interp(t, p2, tq) - pk, axis=1)
        da = ang_between(rot_interp(t, rot2, tq), rk)
        Ep[i] = np.sqrt(np.mean(dp ** 2))
        Er[i] = np.sqrt(np.mean(da ** 2))
        E[i] = np.sqrt(np.mean(dp ** 2 + (DEPTH * da) ** 2))
    j, jp, jr = int(np.argmin(E)), int(np.argmin(Ep)), int(np.argmin(Er))
    eq = float(np.mean(np.radians(wk)) * abs(dts[j]) * DEPTH * 1e3)
    return dict(dt_ms=dts[j] * 1e3, res_mm=float(E[j] * 1e3), eq_mm=eq,
                dt_pos_ms=dts[jp] * 1e3, dt_rot_ms=dts[jr] * 1e3,
                res_rot_deg=float(np.degrees(Er[jr])))


# ----------------------------------------------------------------------------
# 扫描 & 打印
# ----------------------------------------------------------------------------
HDR = ('%-26s %7s %7s %8s %8s | %6s %6s %7s %7s | %7s %7s %8s %8s | %7s %6s' % (
    '操作点', 'hf_p50', 'hf_p90', 'hfT_50', 'hfT_90',
    'dTh50', 'dTh95', 'dP50', 'dP95',
    'dt*ms', 'res_mm', 'w*dt@5m', 'dtRot', '解析延迟', '|H(-1)'))


def fmt_off(v):
    """最佳时间偏移，带 * 表示已触及 ±300 ms 扫描边界（真实滞后不小于该值）。"""
    return '%6.1f%s' % (v, '*' if abs(v) > 297.0 else ' ')


def run_family(makers, t, p, rot, turnA, turnB, fs, title):
    print('\n' + '=' * W)
    print(title)
    print('=' * W)
    print(HDR)
    print('-' * W)
    print('  (mm)      (mm)             (deg)   (deg)     (deg)  (deg)  (mm)   (mm)'
          '      (ms)    (mm)    (mm)    (ms)      (ms)')
    rows = []
    for name, info, fn in makers:
        p2, rot2 = fn(t, p, rot)
        hp, ht = hf_metrics(t, p2, rot2)
        dv = dev_metrics(p, rot, p2, rot2)
        lg = lag_metrics(t, p, rot, p2, rot2, turnA, half=1)
        lgb = lag_metrics(t, p, rot, p2, rot2, turnB, half=6)
        r = dict(name=name, info=info,
                 hfp50=float(np.median(hp) * 1e3), hfp90=float(np.percentile(hp, 90) * 1e3),
                 htp50=float(np.median(ht)), htp90=float(np.percentile(ht, 90)),
                 nyq=nyquist_gain(info), delay=analytic_delay(info, 1.0 / fs), **dv)
        r.update(lag=lg['dt_ms'], res=lg['res_mm'], eq=lg['eq_mm'],
                 dtrot=lg['dt_rot_ms'], resrot=lg['res_rot_deg'],
                 lagB=lgb['dt_ms'], eqB=lgb['eq_mm'], resB=lgb['res_mm'])
        # 有效延迟：滤波器自身群延迟与「快速转向段实测滞后」取大，防止 DC 群延迟为 0
        # 但换向段实际滞后几百毫秒的谐振型滤波器混进 100 ms 预算
        r['lat_eff'] = max(r['delay'], abs(r['lag']))
        rows.append(r)
        print('%-26s %7.3f %7.3f %8.4f %8.4f | %6.3f %6.3f %7.2f %7.2f | %7s %7.2f %8.2f %7s | %7.1f %6s' % (
            name, r['hfp50'], r['hfp90'], np.degrees(r['htp50']), np.degrees(r['htp90']),
            dv['ang_p50'], dv['ang_p95'], dv['pos_p50'], dv['pos_p95'],
            fmt_off(r['lag']), r['res'], r['eq'], fmt_off(r['dtrot']), r['delay'],
            '  -  ' if r['nyq'] != r['nyq'] else '%6.4f' % r['nyq']))
    return rows


def pareto(rows, kinds, label, extra=''):
    cand = sorted([r for r in rows if r['info']['kind'] in kinds and r['htp50'] == r['htp50']],
                  key=lambda x: (x['htp50'], x['delay']))
    front, best = [], float('inf')
    for r in cand:
        if r['delay'] < best - 1e-9:
            front.append(r)
            best = r['delay']
    print('\nPareto 前沿（%s）：目标 = hfθ_p50 越小越好 且 算法延迟越小越好%s' % (label, extra))
    print('  %-26s %9s %10s %9s %11s %11s' % (
        '操作点', 'hfθ_p50°', '解析延迟ms', 'dt*ms', 'w*dt@5m mm', '贴合残差mm'))
    for r in front:
        print('  %-26s %9.4f %10.1f %9s %11.2f %11.2f' % (
            r['name'], np.degrees(r['htp50']), r['delay'], fmt_off(r['lag']), r['eq'], r['res']))
    if len(front) < 2:
        print('  （前沿只有 1 点：该族延迟恒为 0，任何点都被「抖动最小」那点支配。'
              '零相位族因此只能当上限参照，不构成可选型，请看完整表。）')
    return front


# ----------------------------------------------------------------------------
# 第四步：GT 上的 ATE
# ----------------------------------------------------------------------------
def ate_ntu(cands, fs_ref):
    """NTU VIRAL eee_03。base = run_ntu_off（Super-LIO 关视觉 = 纯 LIO 后验，1617 帧 @~10Hz）。
       对齐协议沿用仓库 tools/odom_vs_gt.py：prism 杆臂补偿 -> GT 插值 -> SE(3) Umeyama。"""
    sys.path.insert(0, '/home/ywj/git/Super-LIVO/tools')
    try:
        import odom_vs_gt as og
        est = og.load_odom(NTU_BAG, '/lio/odom', 'header')
        gt = og.load_gt(NTU_GT)
    except Exception as ex:
        print('  [NTU] GT-ATE 无法执行：%r' % ex)
        return
    t, p, rot = est[:, 0], est[:, 1:4], RO(est[:, 4:8])
    hp, ht = hf_metrics(t, p, rot)
    print('\n  [NTU eee_03 / run_ntu_off] n=%d 时长 %.0f s dt 中位 %.1f ms  >0.3s 空隙 %d 处' % (
        len(t), t[-1] - t[0], np.median(np.diff(t)) * 1e3, int((np.diff(t) > GAP_MAX).sum())))
    print('  原始抖动 hf_p P50=%.2f mm  hf_θ P50=%.1f mdeg' % (
        np.median(hp) * 1e3, mdeg(np.median(ht))))
    r0 = og.evaluate('raw(LIO)', est.copy(), gt)
    if r0 is None:
        print('  GT-ATE 无法执行：估计与真值时间不重叠')
        return
    ate0 = r0['ate']
    out = [('raw(LIO)', ate0, max(r0['qs']))]
    for name, info, fn in cands:
        p2, rot2 = fn(t, p, rot)
        r2 = og.evaluate(name, np.column_stack([t, p2, rot2.as_quat()]), gt)
        if r2 is None:
            continue
        out.append((name, r2['ate'], max(r2['qs'])))
    print('\n  ATE 变化汇总（对齐后位置误差 RMSE，m）：')
    print('  %-26s %9s %10s %9s %10s' % ('操作点', 'ATE(m)', 'ΔATE(mm)', '相对', '最差分段(m)'))
    for name, v, worst in out:
        print('  %-26s %9.4f %+10.1f %+9.1f%% %10.3f' % (
            name, v, (v - ate0) * 1e3, (v / ate0 - 1) * 100, worst))
    return ate0


def ate_rtk(cands):
    """RTK-SLAM construction_seq2。base = super_lio 离线轨迹（5867 帧 @~10Hz）。
       GT 只有 16 个测绘检查点（位置），对齐用 SE(3)（不用 Sim(3)，否则平滑的尺度改动会被吸收）。"""
    sys.path.insert(0, RTK_ROOT + '/eval')
    try:
        from readers import read_ground_truth
        from coords import convert_extracted_points_to_utm
        a = np.loadtxt(RTK_TRAJ)
        gtp = read_ground_truth(RTK_GTCSV)
        origin = json.loads(open(RTK_ORIGIN).read())
    except Exception as ex:
        print('  [RTK] GT-ATE 无法执行（缺 %s）：%r' % (RTK_ROOT, ex))
        return
    t, p, rot = a[:, 0], a[:, 1:4], RO(a[:, 4:8])
    hp, ht = hf_metrics(t, p, rot)

    def ate_of(pp):
        idx = []
        for g in gtp:
            j = int(np.argmin(np.abs(t - g['timestamp'])))
            if abs(t[j] - g['timestamp']) <= 2.0:
                idx.append((j, g['position']))
        pts = [{'position': np.asarray(pp[j], float)} for j, _ in idx]
        utm = convert_extracted_points_to_utm(pts, origin)
        src = np.array([u['position'] for u in utm])
        dst = np.array([gp for _, gp in idx])
        sc, dc = src.mean(0), dst.mean(0)
        U, _, Vt = np.linalg.svd((src - sc).T @ (dst - dc))
        Rr = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
        e = src @ Rr.T + (dc - Rr @ sc) - dst
        return float(np.sqrt(np.mean(np.sum(e ** 2, 1)))), len(idx)

    ate0, nchk = ate_of(p)
    print('\n  [RTK-SLAM construction_seq2 / super_lio] n=%d 时长 %.0f s dt 中位 %.1f ms '
          '检查点 %d 个' % (len(t), t[-1] - t[0], np.median(np.diff(t)) * 1e3, nchk))
    print('  原始抖动 hf_p P50=%.2f mm  hf_θ P50=%.1f mdeg  原始 SE(3) ATE=%.3f m' % (
        np.median(hp) * 1e3, mdeg(np.median(ht)), ate0))
    print('  %-26s %9s %10s %10s' % ('操作点', 'ATE(m)', 'ΔATE(mm)', '相对'))
    print('  %-26s %9.4f %10s %10s' % ('raw(LIO)', ate0, '-', '-'))
    for name, info, fn in cands:
        p2, _ = fn(t, p, rot)
        v, _ = ate_of(p2)
        print('  %-26s %9.4f %+10.1f %+9.1f%%' % (name, v, (v - ate0) * 1e3, (v / ate0 - 1) * 100))
    return ate0


# ----------------------------------------------------------------------------
def main():
    t, p, rot = load_tum(BASE)
    N = len(t)
    dt = np.diff(t)
    fs = 1.0 / float(np.median(dt))

    print('#' * W)
    print('# P0-B 纯位姿平滑判决实验（离线复现，无 GPU / 无 ROS）')
    print('#' * W)
    print('输入 %s' % BASE)
    print('  n=%d  时长 %.1f s  dt 中位 %.2f ms  标称 %.2f Hz  >%.2f s 空隙 %d 处(帧号 %s)' % (
        N, t[-1] - t[0], np.median(dt) * 1e3, fs, GAP_MAX,
        int((dt > GAP_MAX).sum()), (np.where(dt > GAP_MAX)[0] + 1).tolist()))

    # ---------------- 第一步：锚点复现 ----------------
    hp, ht = hf_metrics(t, p, rot)
    got = dict(hp_p50=np.median(hp) * 1e3, hp_p90=np.percentile(hp, 90) * 1e3,
               hp_max=hp.max() * 1e3, ht_p50=mdeg(np.median(ht)),
               ht_p90=mdeg(np.percentile(ht, 90)))
    print('\n' + '=' * W)
    print('[第一步] 3-tap 二阶差分口径 —— 锚点复现')
    print('=' * W)
    print('  hf_p[k] = |p_k - (p_{k-1}+p_{k+1})/2|   (mm)')
    print('  hf_T[k] = |Log( slerp(R_{k-1},R_{k+1},0.5)^T R_k )|   (mdeg)')
    print('  有效帧 %d / %d（跨 >%.2f s 空隙的 k 已作废）' % (valid_mask(t).sum(), N - 2, GAP_MAX))
    print('  %-10s %10s %12s %9s' % ('指标', '锚点', '本次复现', '偏差'))
    for k, lab in [('hp_p50', 'hf_p P50'), ('hp_p90', 'hf_p P90'), ('hp_max', 'hf_p max'),
                   ('ht_p50', 'hf_θ P50'), ('ht_p90', 'hf_θ P90')]:
        print('  %-10s %10.2f %12.2f %8.2f%%' % (lab, ANCHOR[k], got[k],
                                                 (got[k] / ANCHOR[k] - 1) * 100))
    worst = max(abs(got[k] / ANCHOR[k] - 1) for k in ANCHOR) * 100
    print('  最大偏差 %.2f%% -> %s（容差 5%%）' % (worst, '复现通过' if worst <= 5 else '复现失败'))
    if worst > 5:
        sys.exit('口径与锚点不符，后续结论无效')

    w1 = ang_rate(t, rot, 1)     # 0.2 s 中心基线（定义 A，含部分抖动）
    w6 = ang_rate(t, rot, 6)     # 1.2 s 中心基线（近似真实转向，抖动基本抵消）
    turnA = w1 > TURN_TH
    turnB = w6 > TURN_TH
    print('\n  角速度分布：0.2s 基线 P50=%.1f P90=%.1f max=%.1f deg/s；'
          '1.2s 基线 P50=%.1f P90=%.1f max=%.1f deg/s' % (
              np.median(w1), np.percentile(w1, 90), w1.max(),
              np.median(w6), np.percentile(w6, 90), w6.max()))
    print('  逐帧增量幅值 P50=%.3f°（即 ~%.1f°/s 的高频抖动）；'
          '相对首帧最大偏角 %.1f°' % (
              np.degrees(np.median(np.linalg.norm(increments(rot), axis=1))),
              np.degrees(np.median(np.linalg.norm(increments(rot), axis=1))) / np.median(dt),
              np.degrees(ang_between(rot[0], rot).max())))
    print('  快速转向帧：A(|ω_0.2s|>%.0f°/s) = %d 帧；B(|ω_1.2s|>%.0f°/s) = %d 帧' % (
        TURN_TH, int(turnA.sum()), TURN_TH, int(turnB.sum())))

    # ---------------- 第二步 A：离线零相位 ----------------
    mA = [mk_butter_ff(fc, fs) for fc in (0.2, 0.5, 1.0, 1.5, 2.0)]
    mA += [mk_savgol_center(w, po) for w in (5, 7, 9, 11, 13) for po in (2, 3)]
    rowsA = run_family(mA, t, p, rot, turnA, turnB, fs,
                       '[第二步 A] 离线零相位（filtfilt / 中心窗口 SG）—— '
                       '仅作参照上限，中心 SG w=5/7/9 引入 ~200/300/400 ms，不满足 100 ms 硬约束')

    # ---------------- 第二步 B：在线因果 ----------------
    alphas = [0.05, 0.07, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60,
              0.70, 0.80, 0.85, 0.90, 0.94, 0.96, 0.98, 0.99, 0.995, 0.999]
    lams = [0.001, 0.003, 0.01, 0.02, 0.03, 0.05, 0.08, 0.10, 0.15, 0.20,
            0.25, 1.0 / 3.0, 0.40, 0.50, 0.70, 1.00, 1.50, 2.00, 3.00, 5.00, 10.00]
    mB = [mk_ema(a) for a in alphas] + [mk_minvar2(l) for l in lams]
    rowsB = run_family(mB, t, p, rot, turnA, turnB, fs,
                       '[第二步 B] 在线因果（EMA 一阶低通 / 二阶前向差分正则因果最小方差）—— 真正的候选')

    # ---------------- 第二步 C：固定延迟滑窗 ----------------
    mC = [mk_window_poly(w, o) for w in (3, 5, 7, 9) for o in (1, 2, 3) if o < w - 1]
    rowsC = run_family(mC, t, p, rot, turnA, turnB, fs,
                       '[第二步 C] 允许固定延迟的因果滑窗最小二乘（输出窗口最老一帧），延迟 = (W-1)x100 ms')

    rows = rowsA + rowsB + rowsC

    print('\n' + '=' * W)
    print('[第三步] 列含义')
    print('=' * W)
    print('  hf_p50/hf_p90 : 平滑后重算的 3-tap 平移抖动 (mm)')
    print('  hfT_50/hfT_90 : 平滑后重算的 3-tap 旋转抖动 (deg)')
    print('  dTh50/dTh95   : |Log(R_orig^T R_smooth)| 与原始 LIO 的姿态差 (deg) —— 修正量幅度')
    print('  dP50/dP95     : |p_orig - p_smooth| (mm)')
    print('  dt*           : 快速转向段 A 上，平滑轨迹需前移多少时间才最贴合原轨迹 (ms，正=滞后)')
    print('  res_mm        : 在 dt* 处对齐后的 5 m 深度等效位移残差 RMS (mm)（平滑真的错掉多少）')
    print('  w*dt@5m       : mean|ω|·dt* 换算到 5 m 深度的切向位移 (mm) —— 判决用的滞后等效位移')
    print('  dtRot         : 仅用姿态角差求出的最佳时间偏移 (ms)')
    print('  解析延迟      : 该滤波器本身的算法延迟 (ms) = DC 群延迟 / 窗口延迟')
    print('  |H(-1)|       : 传递函数在 Nyquist 处的增益，解释逐帧交替抖动被压掉的比例')
    print('  dt*/dtRot 带 * : 已触及 ±300 ms 扫描边界，真实滞后不小于该值，此时以解析延迟为准')
    print('  注意 B2（二阶前向差分正则）的 DC 群延迟天然接近 0（对匀速段确实零滞后），')
    print('       但换向/曲率段的实际滞后由实测 dt* 给出，两个都要看才算诚实。')
    print('  原始 LIO 基线：hf_p P50=%.3f mm  hf_θ P50=%.4f deg  解析延迟=0 ms' % (
        got['hp_p50'], got['ht_p50'] / 1e3))

    pareto(rows, {'causal-1st', 'causal-2nd'}, '在线因果 B —— 唯一可部署')
    pareto(rows, {'fixed-delay'}, '固定延迟滑窗 C', '（延迟 = (W-1)x100 ms，超过 100 ms 预算的已标注）')
    pareto(rows, {'offline-ff', 'offline-center'}, '离线零相位 A', '（参照上限，不可部署）')

    print('\n在线因果候选完整表（按 hfθ_p50 升序，覆盖任务要求的 [0.05°, 0.51°] 区间）')
    print('  %-26s %9s %9s %10s %9s %11s %11s' % (
        '操作点', 'hfθ_p50°', 'hf_p50mm', '解析延迟ms', 'dt*ms', 'w*dt@5m mm', '贴合残差mm'))
    for r in sorted([x for x in rows if x['info']['kind'] in ('causal-1st', 'causal-2nd')],
                    key=lambda x: x['htp50']):
        flag = ''
        if r['delay'] > 100.0:
            flag = '  <== 解析延迟超 100 ms 预算'
        elif r['lat_eff'] > 100.0:
            flag = '  <== 解析延迟达标但实测转向段滞后超预算'
        print('  %-26s %9.4f %9.3f %10.1f %9s %11.2f %11.2f%s' % (
            r['name'], np.degrees(r['htp50']), r['hfp50'], r['delay'], fmt_off(r['lag']),
            r['eq'], r['res'], flag))

    # ---------------- 第五步：判决 ----------------
    print('\n' + '=' * W)
    print('[第五步] P0-B 判决：是否存在「在线因果」操作点同时满足 '
          'hfθ_p50 <= %.2f° 且 快速转向段滞后等效位移 <= %.0f mm' % (THETA_JUDGE, EQ_JUDGE))
    print('=' * W)
    online = [r for r in rows if r['info']['kind'] in ('causal-1st', 'causal-2nd')]

    def gate(pool, tag, eqkey='eq'):
        """在给定在线因果子集里判「有没有达标点」，没有就说清卡在哪个指标、差多少。"""
        if not pool:
            print('  [%s] 子集为空（无点满足延迟预算）' % tag)
            return []
        hit = [r for r in pool
               if np.degrees(r['htp50']) <= THETA_JUDGE and r[eqkey] <= EQ_JUDGE]
        print('  [%s] 候选 %d 个 -> %s' % (tag, len(pool),
              '存在 %d 个达标点' % len(hit) if hit else '不存在达标点'))
        for r in hit:
            print('      达标: %-26s hfθ_p50=%.4f°  滞后等效=%.2f mm  延迟=%.1f ms  残差=%.2f mm' % (
                r['name'], np.degrees(r['htp50']), r[eqkey], r['delay'], r['res']))
        if hit:
            return hit
        g1 = [r for r in pool if np.degrees(r['htp50']) <= THETA_JUDGE]
        if g1:
            b = min(g1, key=lambda x: x[eqkey])
            print('      卡在滞后：hfθ_p50<=%.2f° 的点里 %s 的滞后等效位移最小 = %.2f mm'
                  '（判据 %.0f mm，超 %.1f 倍）' % (
                      THETA_JUDGE, b['name'], b[eqkey], EQ_JUDGE, b[eqkey] / EQ_JUDGE))
        else:
            b = min(pool, key=lambda x: x['htp50'])
            print('      卡在抖动：hfθ_p50 最小 = %s 的 %.4f°（判据 %.2f°，差 %.1f 倍）' % (
                b['name'], np.degrees(b['htp50']), THETA_JUDGE,
                np.degrees(b['htp50']) / THETA_JUDGE))
        g2 = [r for r in pool if r[eqkey] <= EQ_JUDGE]
        if g2:
            b2 = min(g2, key=lambda x: x['htp50'])
            print('      反向：滞后等效位移<=%.0f mm 的点里 hfθ_p50 最小 = %.4f°（判据 %.2f°，'
                  '差 %.1f 倍）' % (EQ_JUDGE, np.degrees(b2['htp50']), THETA_JUDGE,
                                    np.degrees(b2['htp50']) / THETA_JUDGE))
        else:
            b2 = min(pool, key=lambda x: x[eqkey])
            print('      反向：滞后等效位移最小 = %s 的 %.2f mm（判据 %.0f mm，差 %.1f 倍）' % (
                b2['name'], b2[eqkey], EQ_JUDGE, b2[eqkey] / EQ_JUDGE))
        return []

    gate([r for r in online if r['delay'] <= 100.0], '口径1 判据=解析/DC 群延迟<=100ms')
    gate([r for r in online if r['lat_eff'] <= 100.0],
         '口径2 判据=解析延迟与实测滞后均<=100ms')
    # 放宽到完全不限延迟
    print('  参照（取消延迟约束、所有在线因果点）：')
    gate(online, '不限延迟')
    # 稳健性复核
    print('\n  稳健性复核（改用 1.2 s 基线角速度定义的真实快速转向段 B，共 %d 帧）：'
          % int(turnB.sum()))
    gate([r for r in online if r['lat_eff'] <= 100.0], '口径2 + 转向段 B', eqkey='eqB')

    # ---------------- 第四步：GT-ATE ----------------
    print('\n' + '=' * W)
    print('[第四步] GT 序列上的 ATE 不劣化检查（SE(3) 对齐；不用 Sim(3)，'
          '否则会吸掉平滑引入的尺度变化）')
    print('=' * W)

    def build(r):
        i = r['info']
        return (mk_ema(i['alpha']) if i['kind'] == 'causal-1st' else
                mk_minvar2(i['lam']) if i['kind'] == 'causal-2nd' else
                mk_window_poly(i['win'], i['order']) if i['kind'] == 'fixed-delay' else
                mk_butter_ff(i['fc'], fs) if i['kind'] == 'offline-ff' else
                mk_savgol_center(i['win'], i['poly']))

    keep = sorted([r for r in rows if r['info']['kind'] in
                   ('causal-1st', 'causal-2nd', 'fixed-delay')],
                  key=lambda x: x['htp50'])
    picks = keep[:4]
    for target in (0.15, 0.30):
        picks.append(min(keep, key=lambda x: abs(np.degrees(x['htp50']) - target)))
    for dref in (50.0, 100.0):
        picks.append(min(keep, key=lambda x: abs(x['delay'] - dref)))
    # 每个在线因果族各自「100 ms 延迟预算内抖动最好」的点必须送检：
    # 否则 B2 只会以 λ->0（等于不做任何平滑）被挑中，无法回答「真的平滑了会不会毁 ATE」
    for kk in ('causal-1st', 'causal-2nd'):
        budget = [r for r in online if r['info']['kind'] == kk and r['lat_eff'] <= 100.0]
        if budget:
            picks.append(min(budget, key=lambda x: x['htp50']))
    pool = rows                       # rows 里不含 raw 基线行（raw 的 eq/res 恒为 0，会污染 min）
    picks.append(min(pool, key=lambda x: x['eq']))                # 滞后等效位移最小
    picks.append(min(pool, key=lambda x: x['res']))               # 贴合残差最小
    ref = ([r for r in rows if r['info']['kind'] == 'offline-ff'] +
           [r for r in rows if r['info']['kind'] == 'offline-center' and r['info']['poly'] == 2])
    picks.append(min(ref, key=lambda x: x['htp50']))             # 零相位理论上限（仅参照）
    cands, seen = [], set()
    for r in picks:
        if r is None or r['name'] in seen:
            continue
        seen.add(r['name'])
        cands.append(build(r))
    print('  送检操作点（从第三步全表按「抖动最好 / 目标档 / 延迟档 / 预算内最好 / 滞后最小 /'
          ' 零相位上限参照」挑选，共 %d 个）：' % len(cands))
    for nm, inf, _ in cands:
        rr = [x for x in rows if x['name'] == nm][0]
        print('    %-26s 解析延迟=%7.1f ms  hfθ_p50=%.4f°  实测滞后=%s ms' % (
            nm, analytic_delay(inf, 1.0 / fs), np.degrees(rr['htp50']), fmt_off(rr['lag'])))
    if not (os.path.exists(NTU_BAG) or os.path.exists(RTK_TRAJ)):
        print('  GT-ATE 无法执行：候选数据文件都不存在，改用第三步的滞后误差作为真实运动误差。')
    else:
        ate_ntu(cands, fs)
        ate_rtk(cands)

    print('\n' + '#' * W)
    print('# 实验结束。所有数字均为本脚本实测输出。')
    print('#' * W)


if __name__ == '__main__':
    main()
