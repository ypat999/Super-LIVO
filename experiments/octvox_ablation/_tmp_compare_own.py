#!/usr/bin/env python3
"""临时对比：A/B 用 LIO 位姿 vs 用各自视觉修正位姿 上色，与官方 PCD 比空间颜色一致性。"""
import sys

import numpy as np
from scipy.spatial import cKDTree

PATHS = {
    'A_raw': '/home/ywj/slam_data/colored_fl2_20261114/fl2_A/super_livo_visual_a_colored.pcd',
    'B_raw': '/home/ywj/slam_data/colored_fl2_20261114/fl2_B/super_livo_visual_b_colored.pcd',
    'A_own': '/home/ywj/slam_data/colored_fl2_own_20261114/fl2_A/super_livo_visual_a_colored.pcd',
    'B_own': '/home/ywj/slam_data/colored_fl2_own_20261114/fl2_B/super_livo_visual_b_colored.pcd',
    'OFF':   '/home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb_v2.pcd',
}

FIELD_SETS = [
    [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')],
    [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4'), ('_', '<f4')],
]


def load_pcd(path):
    with open(path, 'rb') as f:
        header = []
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError('bad pcd ' + path)
            header.append(line.decode('ascii', 'ignore'))
            if line.strip().startswith(b'DATA'):
                break
        off = f.tell()
    n = None
    fields = []
    for line in header:
        t = line.split()
        if t and t[0] == 'POINTS':
            n = int(t[1])
        if t and t[0] == 'FIELDS':
            fields = t[1:]
    dtype = None
    for cand in FIELD_SETS:
        if [c[0] for c in cand[:len(fields)]] == fields:
            dtype = np.dtype(cand)
            break
    if dtype is None:
        raise RuntimeError('unsupported fields %s in %s' % (fields, path))
    a = np.memmap(path, dtype=dtype, mode='r', offset=off, shape=(n,))
    xyz = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)
    rgb_i = a['rgb'].view(np.uint32)
    rgb = np.stack([(rgb_i & 255), ((rgb_i >> 8) & 255), ((rgb_i >> 16) & 255)],
                   axis=1).astype(np.float32)
    ok = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 0)
    return xyz[ok], rgb[ok]


def voxel_rms(xyz, rgb, vs):
    key = np.floor(xyz / vs).astype(np.int64)
    key = key - key.min(0)
    dims = key.max(0) + 1
    idx = (key[:, 0] * dims[1] + key[:, 1]) * dims[2] + key[:, 2]
    _, inv, cnt = np.unique(idx, return_inverse=True, return_counts=True)
    rms = np.zeros(len(cnt))
    for c in range(3):
        s = np.bincount(inv, weights=rgb[:, c], minlength=len(cnt))
        s2 = np.bincount(inv, weights=rgb[:, c].astype(np.float64) ** 2, minlength=len(cnt))
        rms += np.maximum(s2 / cnt - (s / cnt) ** 2, 0)
    return cnt, np.sqrt(rms / 3.0)


def nn_metric(xyz, rgb, n_query=1_000_000, k=7, radius=0.05, seed=0):
    rng = np.random.default_rng(seed)
    tree = cKDTree(xyz)
    q = rng.choice(len(xyz), size=min(n_query, len(xyz)), replace=False)
    dist, idx = tree.query(xyz[q], k=k, workers=-1)
    valid = dist[:, 1:] < radius
    neigh = rgb[idx[:, 1:]]
    dev = np.abs(neigh - rgb[q][:, None, :]).mean(axis=2)
    dev = np.where(valid, dev, np.nan)
    per_pt = np.nanmean(dev, axis=1)
    per_pt = per_pt[np.isfinite(per_pt)]
    return per_pt.mean(), (per_pt > 80).mean() * 100.0, len(per_pt)


print('%-6s %10s %8s %8s %8s %9s %9s' %
      ('cloud', 'points', 'rms2cm', 'rms5cm', 'nn_mean', 'nn>80%', 'covered'))
for name, path in PATHS.items():
    try:
        xyz, rgb = load_pcd(path)
    except FileNotFoundError:
        print('%-6s MISSING %s' % (name, path))
        continue
    c2, r2 = voxel_rms(xyz, rgb, 0.02)
    c5, r5 = voxel_rms(xyz, rgb, 0.05)
    nn_mean, nn_bad, nn_n = nn_metric(xyz, rgb)
    print('%-6s %10d %8.2f %8.2f %8.2f %8.2f%% %9d' %
          (name, len(xyz), r2[c2 >= 2].mean(), r5[c5 >= 2].mean(), nn_mean, nn_bad, nn_n))
    del xyz, rgb

# 同一 1cm 体素上，raw 与 own 的颜色差异（判断开关实际改变了多少）
def keyed_color(xyz, rgb, vs=0.01):
    key = np.floor(xyz / vs).astype(np.int64)
    h = (key[:, 0] * 73856093) ^ (key[:, 1] * 19349663) ^ (key[:, 2] * 83492791)
    order = np.argsort(h, kind='stable')
    h, rgb_s = h[order], rgb[order]
    uniq, start = np.unique(h, return_index=True)
    mu = np.stack([np.add.reduceat(rgb_s[:, c], start) for c in range(3)], axis=1)
    cnt = np.diff(np.append(start, len(h)))
    return uniq, mu / cnt[:, None]

for plan in ('A', 'B'):
    try:
        x1, c1 = load_pcd(PATHS['%s_raw' % plan])
        x2, c2 = load_pcd(PATHS['%s_own' % plan])
    except FileNotFoundError:
        continue
    k1, m1 = keyed_color(x1, c1)
    k2, m2 = keyed_color(x2, c2)
    common, i1, i2 = np.intersect1d(k1, k2, return_indices=True)
    d = np.abs(m1[i1] - m2[i2]).mean(axis=1)
    print('\n[%s] 共有 1cm 体素 %d: 颜色平均变化 mean=%.2f, frac>20=%.2f%%, frac>60=%.2f%%'
          % (plan, len(common), d.mean(), (d > 20).mean() * 100, (d > 60).mean() * 100))
    del x1, x2, c1, c2, k1, k2, m1, m2, common, i1, i2, d

sys.stdout.flush()
