#!/usr/bin/env python3
"""误差是有界随机散布，还是一段可被刚体变换吸收的系统偏置？

对每个环境的检查点子集分别拟合：
  (a) 仅平移（均值误差）
  (b) 完整 SE(3)（Umeyama，允许旋转）
看残差掉多少。若掉一大半，说明该段误差本质是"整段被平移/转了一点"，
不是逐帧随机散布；这也决定了该往初始化/退化方向查，而不是往抖动方向查。
"""
import sys
from pathlib import Path

import numpy as np

R = Path('/home/ywj/slam_data/rtk_slam/eval_ref')
sys.path.insert(0, str(R / 'eval'))
sys.path.insert(0, '/home/ywj/git/Super-LIVO/tools')
from readers import read_tum_trajectory, read_ground_truth  # noqa: E402
from coords import convert_extracted_points_to_utm  # noqa: E402
import json  # noqa: E402

SEQ = 'construction_seq2'


def umeyama(src, dst):
    """返回把 src 变到 dst 的 SE(3)（无尺度）。"""
    ms, md = src.mean(0), dst.mean(0)
    S, D = src - ms, dst - md
    cov = D.T @ S / len(src)
    U, dV, Vt = np.linalg.svd(cov)
    Sg = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        Sg[2, 2] = -1
    Rm = U @ Sg @ Vt
    tm = md - Rm @ ms
    return Rm, tm


def main():
    methods = sys.argv[1:] or ['super_lio', 'super_lio_beam', 'fast_lio_sam']
    gt = read_ground_truth(str(R / 'ground_truth' / f'{SEQ}.csv'))
    for m in methods:
        d = R / 'trajectories' / m / SEQ
        origin = json.loads((d / 'enu_origin.json').read_text())
        ts, pos = read_tum_trajectory(str(d / 'traj_offline.txt'))
        rows = []
        for p in gt:
            j = int(np.argmin(np.abs(ts - p['timestamp'])))
            if abs(ts[j] - p['timestamp']) > 0.05:
                continue
            u = convert_extracted_points_to_utm([{'position': pos[j]}], origin)[0]['position']
            rows.append((p['env'], np.array(u), p['position']))
        print(f'\n== {m} ==（{len(rows)} 个检查点匹配 |dt|<=50ms）')
        print(f'{"env":12s}{"n":>3s}{"原始 rms":>10s}{"去平移":>9s}{"去 SE3":>8s}{"均误差[E,N,U]":>22s}')
        for env in ('outdoor', 'transition', 'indoor'):
            sub = [r for r in rows if r[0] == env]
            if len(sub) < 3:
                continue
            E = np.array([r[1] for r in sub])
            G = np.array([r[2] for r in sub])
            e = E - G
            rms0 = np.sqrt((e ** 2).sum(1).mean())
            mean = e.mean(0)
            rms1 = np.sqrt(((e - mean) ** 2).sum(1).mean())
            Rm, tm = umeyama(G, E)
            rms2 = np.sqrt((((Rm @ G.T).T + tm - E) ** 2).sum(1).mean())
            flag = '  (n<=6，6DoF 近乎自由拟合，仅去平移可信)' if len(sub) <= 6 else ''
            print(f'{env:12s}{len(sub):3d}{rms0:10.3f}{rms1:9.3f}{rms2:8.3f}'
                  f'   [{mean[0]:+6.2f},{mean[1]:+6.2f},{mean[2]:+6.2f}] m{flag}')


if __name__ == '__main__':
    main()
