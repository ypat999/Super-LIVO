#!/usr/bin/env python3
"""逐帧高频抖动（无需稠密真值、可跨 run 比较的相对度量）。

先插值到均匀 10 Hz 网格（跳过任何相邻帧间隔 >0.3 s 的窗口），再做 3 tap
二阶差分 hf[k] = p[k] - (p[k-1] + p[k+1])/2 ：匀速直线 hf=0，只有真实平台的
HF 运动和位姿随机抖动才会产生 hf。所有 run 跑同一份 bag，真实运动分量相同，
因此 **run 之间的 hf 差可直接归因于位姿估计抖动**（这也是室内可用的唯一途径）。

⚠️ 只能用于"同一节点的不同 run"之间比较。官方基线轨迹是 ~1 Hz 导出，
插值到 10 Hz 会被人为抹平，不可与本工具的 Super-LIO 数字并列。
"""
import json
import sys
from pathlib import Path

import numpy as np

R = Path('/home/ywj/slam_data/rtk_slam/eval_ref')
sys.path.insert(0, str(R / 'eval'))
sys.path.insert(0, '/home/ywj/git/Super-LIVO/tools')
from readers import read_tum_trajectory  # noqa: E402
from rtk_traj_prep import read_gnss  # noqa: E402

SEQ = 'construction_seq2'
SRC_BAG = '/home/ywj/slam_data/rtk_slam/construction_seq2_r2'
GAP_MAX = 0.3


def grid_pos(ts, pos, dt=0.1):
    """插值到均匀网格；返回 (t_grid, pos_grid, ok_mask)。"""
    g = np.arange(ts[0], ts[-1], dt)
    k = np.searchsorted(ts, g).clip(1, len(ts) - 1)
    lo, hi = k - 1, k
    w = (g - ts[lo]) / (ts[hi] - ts[lo])
    p = pos[lo] + (pos[hi] - pos[lo]) * w[:, None]
    # 网格点若落在任何 >GAP_MAX 的原始空隙内则作废
    big = np.where(np.diff(ts) > GAP_MAX)[0]
    ok = np.ones(len(g), bool)
    for b in big:
        ok &= ~((g > ts[b]) & (g < ts[b + 1]))
    return g, p, ok


def main():
    methods = sys.argv[1:] or ['super_lio', 'super_lio_beam']
    gts, gllh, gst = read_gnss(SRC_BAG)
    fixed = gst >= 0

    for m in methods:
        d = R / 'trajectories' / m / SEQ
        origin = json.loads((d / 'enu_origin.json').read_text())
        ts, pos = read_tum_trajectory(str(d / 'traj_offline.txt'))
        raw_dt = np.diff(ts)
        g, p, ok = grid_pos(ts, pos)
        L = len(g)
        # 以下所有数组统一对齐到网格下标 k = 1 .. L-2
        hk = np.linalg.norm(p[1:-1] - 0.5 * (p[:-2] + p[2:]), axis=1)
        okk = ok[:-2] & ok[1:-1] & ok[2:]
        gg = g[1:-1]
        j = np.searchsorted(gts, gg).clip(1, len(gts) - 1)
        lo, hi = j - 1, j
        best = np.where(np.abs(gts[lo] - gg) < np.abs(gts[hi] - gg), lo, hi)
        infix = fixed[best] & (np.abs(gts[best] - gg) < 0.15)

        print(f'== {m} ==  帧 {len(ts)}，原始 dt 中位 {np.median(raw_dt) * 1000:.1f} ms / '
              f'>0.3s 空隙 {(raw_dt > GAP_MAX).sum()} 处；网格有效点 {okk.sum()}')

        def rep(mask, label):
            v = hk[mask & okk]
            if len(v) < 30:
                print(f'   {label:22s} n={len(v):5d}  —')
                return
            print(f'   {label:22s} n={len(v):5d}  |hf| 中位 {np.median(v) * 100:4.1f}  '
                  f'p90 {np.percentile(v, 90) * 100:5.1f}  p99 {np.percentile(v, 99) * 100:6.1f} cm  '
                  f'>5cm {100 * (v > 0.05).mean():4.1f}%  >20cm {100 * (v > 0.2).mean():4.1f}%')
        rep(infix, 'RTK 可核验段（室外）')
        rep(~infix, '无 fix（室内为主）')
        rep(np.ones(L - 2, bool), '全程')
        print('   注：不要用 ±秒级"去趋势残差"当抖动指标——10 s 窗口内真实转弯的'
              '弓高就有 0.3~1 m，会被完全淹没。逐帧 hf 才是可用量。')

        # ---- 原始帧路径长度（含跨空隙的大步），按 RTK 有无 fix 拆分 ----
        sn = np.linalg.norm(np.diff(pos, axis=0), axis=1)
        dt = np.diff(ts)
        mid = 0.5 * (ts[:-1] + ts[1:])
        jj = np.searchsorted(gts, mid).clip(1, len(gts) - 1)
        l_, h_ = jj - 1, jj
        bb = np.where(np.abs(gts[l_] - mid) < np.abs(gts[h_] - mid), l_, h_)
        f_ = fixed[bb] & (np.abs(gts[bb] - mid) < 0.2)
        big = dt > GAP_MAX
        print('   --- 原始帧路径 ---')
        for lab, msk in (('RTK 可核验（室外）', f_),
                         ('无 fix（室内为主）', ~f_),
                         ('全程', np.ones(len(sn), bool))):
            v = sn[msk]
            print(f'   {lab:22s} {v.sum():7.1f} m  跨空隙步 {sn[msk & big].sum():6.1f} m  '
                  f'中位步长 {np.median(v) * 100:5.1f} cm  有效时长 {dt[msk].sum():5.0f} s')


if __name__ == '__main__':
    main()
