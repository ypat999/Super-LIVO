#!/usr/bin/env python3
"""临时对比：Super-LIO 位姿 + 官方染色 vs 其它三条（跑完即删）。"""
import sys

import numpy as np
from scipy.spatial import cKDTree

PATHS = {
    'SL_OFF': '/home/ywj/slam_data/colored_superlio_official_20261114/superlio_official_rgb.pcd',
    'SL_A':   '/home/ywj/slam_data/colored_superlio_a_20261114/super_livo_visual_a_colored.pcd',
    'SL_A_R3': '/home/ywj/slam_data/colored_bc_20261110/rep_B/super_livo_visual_a_colored.pcd',
    'FL2_A':  '/home/ywj/slam_data/colored_fl2_20261114/fl2_A/super_livo_visual_a_colored.pcd',
    'OFF':    '/home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb_v2.pcd',
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


print('%-7s %11s %8s %8s %8s %9s %9s' %
      ('cloud', 'points', 'rms2cm', 'rms5cm', 'nn_mean', 'nn>80%', 'covered'))
for name, path in PATHS.items():
    try:
        xyz, rgb = load_pcd(path)
    except FileNotFoundError:
        print('%-7s MISSING %s' % (name, path))
        continue
    c2, r2 = voxel_rms(xyz, rgb, 0.02)
    c5, r5 = voxel_rms(xyz, rgb, 0.05)
    nn_mean, nn_bad, nn_n = nn_metric(xyz, rgb)
    print('%-7s %11d %8.2f %8.2f %8.2f %8.2f%% %9d' %
          (name, len(xyz), r2[c2 >= 2].mean(), r5[c5 >= 2].mean(), nn_mean, nn_bad, nn_n))
    del xyz, rgb

sys.stdout.flush()
