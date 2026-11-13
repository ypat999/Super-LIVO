#!/usr/bin/env python3
"""判断轨迹是"越走越偏"（累积漂移）还是"原地抖"（逐帧随机）。

思路（全部在 UTM 下做，差的量纲与 ENU 等价）：
  1) 每个检查点：误差 e_i = est_i - gt_i，并记录到该时刻为止**已走过的路程** s_i
     （用我们自己的轨迹累积，不是直线距离）；回归 |e| = a + b·s，b 即漂移率。
  2) 检查点两两配对（120 对）：比较相对位移向量 d_est 与 d_gt ——
     若存在尺度/累积误差，|d_est|/|d_gt| 会随 |d_gt| 系统性偏离 1；
     再把相对误差分解为"沿航向(along)"与"垂直航向(cross)"，累积漂移通常
     表现为 along 随距离线性增长，随机抖动则 along/cross 同量级且不随距离增长。
"""
import json
import sys
from pathlib import Path

import numpy as np

R = Path('/home/ywj/slam_data/rtk_slam/eval_ref')
sys.path.insert(0, str(R / 'eval'))
from readers import read_tum_trajectory, read_ground_truth, find_nearest_pose  # noqa: E402
from coords import convert_extracted_points_to_utm  # noqa: E402

SEQ = 'construction_seq2'


def load(method):
    d = R / 'trajectories' / method / SEQ
    origin = json.loads((d / 'enu_origin.json').read_text())
    ts, pos = read_tum_trajectory(str(d / 'traj_offline.txt'))
    gt = read_ground_truth(str(R / 'ground_truth' / f'{SEQ}.csv'))
    # 自己做最近邻匹配（官方 find_nearest_pose 的 index 是 query 下标，且容忍 2 s）
    poses, keep = [], []
    for i, p in enumerate(gt):
        j = int(np.argmin(np.abs(ts - p['timestamp'])))
        if abs(ts[j] - p['timestamp']) <= 0.05:
            poses.append({'index': j, 'position': pos[j]})
            keep.append(i)
    pu = convert_extracted_points_to_utm(poses, origin)
    est = np.array([p['position'] for p in pu], float)
    g = np.array([gt[i]['position'] for i in keep], float)
    idxs = [p['index'] for p in poses]
    return (ts, pos, est, g, [gt[i]['env'] for i in keep],
            [gt[i]['id'] for i in keep], idxs)


def path_len(ts, pos, idx):
    """轨迹起点 -> 第 idx 帧的累积路程。"""
    p = pos[:idx + 1]
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def analyse(name, ts, pos, est, g, envs, ids, idxs):
    e = est - g
    s = np.array([path_len(ts, pos, i) for i in idxs])
    n = np.linalg.norm(e[:, :2], axis=1)                     # 水平误差
    print(f'\n===== {name} =====')
    print(f'{"id":9s} {"env":10s} {"路程s[m]":>9s} {"|e|水平":>8s} {"eE":>7s} {"eN":>7s} {"eU":>7s}')
    for i in range(len(g)):
        print(f'{ids[i]:9s} {envs[i]:10s} {s[i]:9.1f} {n[i]:8.3f} '
              f'{e[i,0]:+7.3f} {e[i,1]:+7.3f} {e[i,2]:+7.3f}')

    # 1) |e| 对路程的线性回归
    A = np.stack([np.ones_like(s), s], 1)
    coef, *_ = np.linalg.lstsq(A, n, rcond=None)
    b_free = coef[1]
    b_fix = float((s @ n) / (s @ s)) if s @ s > 0 else np.nan
    print(f'回归 |e水平| ~ a + b·s : a={coef[0]*100:.1f} cm, b={b_free*1000:.2f} mm/m '
          f'(过原点 b={b_fix*1000:.2f} mm/m)  路程 {s[-1]:.1f} m')
    # 分环境均值
    for env in ('outdoor', 'transition', 'indoor'):
        v = np.array([n[i] for i in range(len(g)) if envs[i] == env])
        if len(v):
            print(f'  {env:11s} n={len(v):2d}  |e|水平 rms={np.sqrt(np.mean(v**2)):.3f} m')

    # 2) 两两相对位移
    k = len(g)
    scale, along, cross, sep = [], [], [], []
    for i in range(k):
        for j in range(i + 1, k):
            dg = g[j] - g[i]
            de = est[j] - est[i]
            L = np.linalg.norm(dg)
            if L < 3.0:
                continue
            u = dg / L
            ve = de - dg
            a = float(ve @ u)
            c = float(np.linalg.norm(ve - a * u))
            scale.append(np.linalg.norm(de) / L)
            along.append(a)
            cross.append(c)
            sep.append(L)
    scale, along, cross, sep = map(np.array, (scale, along, cross, sep))
    print(f'两两相对位移 {len(sep)} 对（基线间隔 3~{sep.max():.0f} m）')
    print(f'  尺度 |d_est|/|d_gt|: 中位 {np.median(scale):.4f}  '
          f'均值 {scale.mean():.4f}  std {scale.std():.4f}')
    print(f'  沿航向残差: 中位 {np.median(np.abs(along))*100:.1f} cm  '
          f'rms {np.sqrt(np.mean(along**2))*100:.1f} cm')
    print(f'  垂直残差  : 中位 {np.median(np.abs(cross))*100:.1f} cm  '
          f'rms {np.sqrt(np.mean(cross**2))*100:.1f} cm')
    # along 是否随间隔增长 = 累积漂移的直接检验
    for lo, hi in ((3, 10), (10, 30), (30, 60), (60, 1e9)):
        m = (sep >= lo) & (sep < hi)
        if m.sum() >= 3:
            print(f'  间隔 {lo:>2.0f}~{hi if hi < 1e8 else 999:>3.0f} m (n={m.sum():3d}): '
                  f'|along|中位 {np.median(np.abs(along[m]))*100:5.1f} cm, '
                  f'|cross|中位 {np.median(np.abs(cross[m]))*100:5.1f} cm, '
                  f'尺度中位 {np.median(scale[m]):.4f}')
    return dict(s=s, n=n, scale=scale, along=along, cross=cross, sep=sep)


def main():
    methods = sys.argv[1:] or ['super_lio', 'fast_lio_sam']
    out = {}
    for m in methods:
        ts, pos, est, g, envs, ids, idxs = load(m)
        out[m] = analyse(m, ts, pos, est, g, envs, ids, idxs)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(11, 4))
        for m in methods:
            d = out[m]
            ax[0].plot(d['s'], d['n'] * 100, 'o-', ms=4, label=m)
            ax[1].plot(d['sep'], np.abs(d['along']) * 100, 'o', ms=3, alpha=.6, label=m + ' along')
            ax[1].plot(d['sep'], np.abs(d['cross']) * 100, 'x', ms=3, alpha=.6, label=m + ' cross')
        ax[0].set_xlabel('travelled distance [m]')
        ax[0].set_ylabel('horizontal |e| [cm]')
        ax[0].set_title('absolute error vs travelled distance')
        ax[0].grid(alpha=.3)
        ax[0].legend()
        ax[1].set_xlabel('checkpoint separation [m]')
        ax[1].set_ylabel('relative error [cm]')
        ax[1].set_title('pairwise relative error (along / cross)')
        ax[1].grid(alpha=.3)
        ax[1].legend(fontsize=7)
        fig.tight_layout()
        p = '/home/ywj/slam_data/rtk_slam/drift_seq2.png'
        fig.savefig(p, dpi=130)
        print('\n图: ' + p)
    except Exception as ex:                                   # noqa: BLE001
        print('作图跳过: %r' % (ex,))


if __name__ == '__main__':
    main()
