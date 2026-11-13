#!/usr/bin/env python3
"""路径长度体检：估计轨迹的"绕路/抖动膨胀" vs RTK 实测长度。

真实运动平稳时，逐帧随机抖动会让累积步长系统性偏大
（E|step_est| ≈ sqrt(L² + σ²)），所以 (L_est - L_ref) 是**逐帧抖动**的
独立度量，与 16 个稀疏检查点互补（检查点只能看有界偏差，看不出抖动）。

只在 RTK 有 fix 的连续时段上做积分，保证两条曲线覆盖同一段路。
"""
import json
import sys
from pathlib import Path

import numpy as np

R = Path('/home/ywj/slam_data/rtk_slam/eval_ref')
sys.path.insert(0, str(R / 'eval'))
sys.path.insert(0, '/home/ywj/git/Super-LIVO/tools')
from readers import read_tum_trajectory  # noqa: E402
from coords import latlon_to_utm, convert_extracted_points_to_utm  # noqa: E402
from rtk_traj_prep import read_gnss  # noqa: E402

SEQ = 'construction_seq2'
SRC_BAG = '/home/ywj/slam_data/rtk_slam/construction_seq2_r2'


def main():
    methods = sys.argv[1:] or ['super_lio', 'super_lio_beam', 'fast_lio_sam']
    gts, gllh, gst = read_gnss(SRC_BAG)
    ok = gst >= 0
    utm = np.array([latlon_to_utm(a, b, c)[:2] for a, b, c in gllh])

    segs, i = [], 0
    while i < len(ok):
        if ok[i]:
            j = i
            while j + 1 < len(ok) and ok[j + 1]:
                j += 1
            if j - i >= 20:
                segs.append((i, j))
            i = j + 1
        else:
            i += 1
    print('RTK 有 fix 段：%d 段，覆盖 %.0f s / %.0f s'
          % (len(segs), sum(gts[b] - gts[a] for a, b in segs), gts[-1] - gts[0]))

    L_ref = sum(np.linalg.norm(np.diff(utm[a:b + 1], axis=0), axis=1).sum() for a, b in segs)
    print('L_ref (RTK) = %.1f m' % L_ref)

    for m in methods:
        d = R / 'trajectories' / m / SEQ
        origin = json.loads((d / 'enu_origin.json').read_text())
        ts, pos = read_tum_trajectory(str(d / 'traj_offline.txt'))
        utm_est = np.array([c['position'] for c in convert_extracted_points_to_utm(
            [{'position': p} for p in pos], origin)])
        L = 0.0
        steps = []
        for a, b in segs:
            t = gts[a:b + 1]
            k = np.searchsorted(ts, t).clip(1, len(ts) - 1)
            lo, hi = k - 1, k
            w = (t - ts[lo]) / (ts[hi] - ts[lo])
            q = utm_est[lo] + (utm_est[hi] - utm_est[lo]) * w[:, None]
            n = np.linalg.norm(np.diff(q[:, :2], axis=0), axis=1)
            L += n.sum()
            steps.append(n)
        steps = np.concatenate(steps)
        excess = L - L_ref
        print('%-16s L=%.1f m  超出 %+.1f m (%+.2f%%)  中位步长 %.3f m  '
              'p99 %.3f m  最大 %.3f m'
              % (m, L, excess, 100 * excess / L_ref, np.median(steps),
                 np.percentile(steps, 99), steps.max()))


if __name__ == '__main__':
    main()
