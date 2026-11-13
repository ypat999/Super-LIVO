#!/usr/bin/env python3
"""逐检查点误差明细 + 分环境 RMSE。

用法：python3 tools/rtk_per_checkpoint.py [method ...]
默认 super_lio fast_lio_sam。所有方法走同一份官方评测代码路径
（同一 ENU 换算、同一天地真值、同一最近邻匹配），因此列之间可直接比较。
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
gt = read_ground_truth(str(R / 'ground_truth' / f'{SEQ}.csv'))
stamps = [p['timestamp'] for p in gt]


def detail(method):
    d = R / 'trajectories' / method / SEQ
    origin = json.loads((d / 'enu_origin.json').read_text())
    ts, pos = read_tum_trajectory(str(d / 'traj_offline.txt'))
    poses = find_nearest_pose(ts, pos, stamps)
    pu = convert_extracted_points_to_utm(poses, origin)
    if len(pu) != len(gt):
        print(f'!! {method}: 只匹配到 {len(pu)}/{len(gt)} 个检查点（|dt|>2s 被丢）')
    rows = []
    for p, cp in zip(pu, poses):
        g = gt[cp['index']]
        e = np.array(p['position']) - g['position']
        rows.append((g['id'], g['env'], float(np.linalg.norm(e)), e))
    return rows


def main():
    methods = sys.argv[1:] or ['super_lio', 'fast_lio_sam']
    W = max(14, max(len(m) for m in methods) + 2)
    data = [detail(m) for m in methods]

    head = f"{'id':9s} {'env':10s}" + ''.join(f'{m:>{W}s}' for m in methods) + '   dENU [m] '
    print(head + ' '.join(f'{m[:6]:>20s}' for m in methods))
    n = min(len(d) for d in data)
    for i in range(n):
        pid, env = data[0][i][0], data[0][i][1]
        line = f'{pid:9s} {env:10s}' + ''.join(f'{d[i][2]:{W}.3f}' for d in data)
        line += '   ' + ' '.join(
            f'{v[3][0]:+6.2f},{v[3][1]:+6.2f},{v[3][2]:+6.2f}' for v in (d[i] for d in data))
        print(line)

    print()
    print(f"{'env':12s}{'n':>3s}" + ''.join(f'{m:>{W}s}' for m in methods))
    for env in ('outdoor', 'transition', 'indoor'):
        idx = [i for i in range(n) if data[0][i][1] == env]
        if not idx:
            continue
        line = f'{env:12s}{len(idx):>3d}'
        for d in data:
            v = [d[i][2] for i in idx if i < len(d)]
            line += f'{np.sqrt(np.mean(np.square(v))):{W}.3f}' if v else f"{'—':>{W}s}"
        print(line)
    line = f"{'ALL':12s}{n:>3d}"
    for d in data:
        v = [r[2] for r in d[:n]]
        line += f'{np.sqrt(np.mean(np.square(v))):{W}.3f}'
    print(line)


if __name__ == '__main__':
    main()
