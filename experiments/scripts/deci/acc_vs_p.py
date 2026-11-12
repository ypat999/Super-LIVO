#!/usr/bin/env python3
"""acc(P) 自适应曲线：逐帧 (Pstd, 是否被 χ² 门接受)。需要 lio.debug.level>=1 的日志。

用法: python3 acc_vs_p.py <log>
"""
import re
import sys

import numpy as np

log = sys.argv[1]
recs = []  # (ts_sec, pnorm_mrad, pz_mrad, chi2, accepted)
for line in open(log, errors='replace'):
    m_t = re.search(r'^[IWEF]\d{4} (\d\d):(\d\d):(\d\d)\.(\d+)', line)
    m_p = re.search(r'Pstd=\[([-\d.eE+]+) ([-\d.eE+]+) ([-\d.eE+]+)\]mrad', line)
    if not (m_t and m_p):
        continue
    sec = int(m_t.group(1)) * 3600 + int(m_t.group(2)) * 60 + int(m_t.group(3)) + int(m_t.group(4)) / 1e6
    p = np.array([float(m_p.group(1)), float(m_p.group(2)), float(m_p.group(3))])
    if '[VisB2] reject' in line:
        m_c = re.search(r'reject chi2=([-\d.eE+]+)', line)
        recs.append((sec, float(np.linalg.norm(p)), p[2], float(m_c.group(1)) if m_c else np.nan, 0))
    elif '[VisRot] inl=' in line:
        m_c = re.search(r' chi2=([-\d.eE+]+) thresh', line)
        recs.append((sec, float(np.linalg.norm(p)), p[2], float(m_c.group(1)) if m_c else np.nan, 1))

if not recs:
    print('  ACCvsP: 无逐帧数据（需 lio.debug.level>=1）')
    sys.exit(0)

recs.sort()
A = np.array([[r[1], r[2], r[3], r[4]] for r in recs])
pn, pz, c2, acc = A[:, 0], A[:, 1], A[:, 2], A[:, 3]
t0 = recs[0][0]
print('  ACCvsP: frames=%d  t0=%.0f  span=%.0fs  总接受=%d/%d(%.1f%%)' % (
    len(recs), t0, recs[-1][0] - t0, int(acc.sum()), len(acc), 100.0 * acc.mean()))

print('  %-14s %6s %7s %9s %9s' % ('|Pstd|区间(mrad)', 'n', 'acc%', 'chi2_med', 'Pz_med'))
edges = [0, 0.8, 1.2, 1.8, 2.5, 3.5, 100]
for lo, hi in zip(edges[:-1], edges[1:]):
    m = (pn >= lo) & (pn < hi)
    if m.sum() < 3:
        continue
    print('  %-14s %6d %6.1f%% %9.1f %9.2f' % (
        '[%.1f,%.1f)' % (lo, hi), m.sum(), 100.0 * acc[m].mean(),
        float(np.median(c2[m])), float(np.median(pz[m]))))

lo = pn < 1.0
hi = pn >= 1.5
if lo.sum() > 3 and hi.sum() > 3:
    print('  低P(|Pstd|<1mrad): n=%d acc=%.1f%% | 高P(|Pstd|>=1.5mrad): n=%d acc=%.1f%% | 差=%+.1f pt' % (
        lo.sum(), 100 * acc[lo].mean(), hi.sum(), 100 * acc[hi].mean(),
        100 * (acc[hi].mean() - acc[lo].mean())))
