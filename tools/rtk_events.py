#!/usr/bin/env python3
"""找轨迹里的"飞行/瞬移"事件：单帧位移远超平台真实速度上限。

RTK 实测本序列最高速度约 1.6 m/s（§19.9 的 GPS 段），因此
|Δp|/Δt > 4 m/s 的帧一定是估计器自己飞出去的，不是运动。
同时检查每个事件是否是"出去又回来"（往返残差小 = 幻觉位移），
这决定它是可丢弃的野值还是真实走偏。
"""
import json
import sys
from pathlib import Path

import numpy as np

R = Path('/home/ywj/slam_data/rtk_slam/eval_ref')
sys.path.insert(0, str(R / 'eval'))
from readers import read_tum_trajectory  # noqa: E402

SEQ = 'construction_seq2'
VMAX = 4.0        # m/s，明显高于 RTK 实测上限


def events(m, show=12):
    d = R / 'trajectories' / m / SEQ
    ts, pos = read_tum_trajectory(str(d / 'traj_offline.txt'))
    t0 = ts[0]
    step = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    dt = np.diff(ts)
    v = step / np.maximum(dt, 1e-6)
    bad = np.where(v > VMAX)[0]
    print(f'== {m} ==  帧 {len(ts)}，路径 {step.sum():.1f} m；'
          f'疑似瞬移帧 {len(bad)}（{100 * len(bad) / len(v):.2f}%），'
          f'这些帧合计 {step[bad].sum():.1f} m（占总路径 {100 * step[bad].sum() / step.sum():.0f}%）')
    if len(bad) == 0:
        return
    # 合并相邻事件成"事件簇"
    clusters, cur = [], [bad[0]]
    for i in bad[1:]:
        if i - cur[-1] <= 3:
            cur.append(i)
        else:
            clusters.append(cur)
            cur = [i]
    clusters.append(cur)
    print(f'   聚成 {len(clusters)} 个事件簇；速度 p50/p90/p99/max = '
          f'{np.percentile(v, 50):.2f}/{np.percentile(v, 90):.2f}/'
          f'{np.percentile(v, 99):.2f}/{v.max():.2f} m/s')
    print(f'   {"t起":>8s} {"时长":>6s} {"簇内步长":>9s} {"回原距离":>9s} {"最大单步":>9s} {"最大dt":>8s}')
    tot_out = 0.0
    info = []
    for c in clusters:
        a, b = c[0], c[-1] + 1
        after = pos[min(b + 20, len(pos) - 1)]
        info.append((step[c].sum(), ts[a] - t0, dt[c].sum(),
                     np.linalg.norm(pos[a] - after), step[c].max(), dt[c].max()))
        tot_out += step[c].sum()
    for s, t, dur, back, mx, md in sorted(info, reverse=True)[:show]:
        print(f'{t:8.1f} {dur:6.2f} {s:9.2f} {back:9.2f} {mx:9.2f} {md * 1000:7.0f}ms')
    print(f'   事件簇合计飞出 {tot_out:.1f} m；回原距离<1 m 的簇 '
          f'{sum(1 for i in info if i[3] < 1.0)}/{len(info)} 个（=纯幻觉位移）')


def main():
    for m in (sys.argv[1:] or ['super_lio', 'super_lio_beam3', 'super_lio_beam']):
        events(m)


if __name__ == '__main__':
    main()
