#!/usr/bin/env python3
"""检查导出 TUM 轨迹的时间戳质量（是否单调、有无重复/倒退）。"""
import sys
from pathlib import Path

import numpy as np

R = Path('/home/ywj/slam_data/rtk_slam/eval_ref')
sys.path.insert(0, str(R / 'eval'))
from readers import read_tum_trajectory  # noqa: E402

for m in (sys.argv[1:] or ['super_lio', 'super_lio_beam3', 'super_lio_beam']):
    ts, pos = read_tum_trajectory(str(R / 'trajectories' / m / 'construction_seq2/traj_offline.txt'))
    dt = np.diff(ts)
    print(f'== {m} ==  {len(ts)} 行，跨度 {ts[-1] - ts[0]:.1f} s')
    print(f'   dt: 负 {(dt < 0).sum()} 个（最小 {dt.min() * 1000:.1f} ms）、'
          f'零 {(dt == 0).sum()} 个、>0.3s {(dt > 0.3).sum()} 个（最大 {dt.max():.2f} s）')
    print(f'   |dt| 倒序幅度合计 {(-dt[dt < 0]).sum():.2f} s；'
          f'按 dt>0 累计 {(dt[dt > 0]).sum():.1f} s')
    # 位置在时间倒退处的跳变
    bad = np.where(dt < 0)[0]
    if len(bad):
        n = np.linalg.norm(np.diff(pos, axis=0), axis=1)
        print(f'   倒退处的位移: 中位 {np.median(n[bad]) * 100:.1f} cm  最大 {n[bad].max() * 100:.1f} cm')
        print(f'   前 5 个倒退时刻（相对秒）: {np.round(ts[bad[:5]] - ts[0], 2)}')
