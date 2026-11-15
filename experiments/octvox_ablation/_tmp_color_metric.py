#!/usr/bin/env python3
"""临时对比脚本：同一 2cm 体素内比较“局部颜色噪声”（跑完即删）。"""
import numpy as np

PATHS = {
    'A': '/home/ywj/slam_data/colored_fl2_20261114/fl2_A/super_livo_visual_a_colored.pcd',
    'B': '/home/ywj/slam_data/colored_fl2_20261114/fl2_B/super_livo_visual_b_colored.pcd',
    'OFF': '/home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb_v2.pcd',
}

def load_pcd(path):
    with open(path, 'rb') as f:
        header = []
        while True:
            line = f.readline()
            header.append(line.decode('ascii', 'ignore'))
            if line.strip().startswith(b'DATA'):
                break
        off = f.tell()
    n = None
    for line in header:
        t = line.split()
        if t and t[0] == 'POINTS':
            n = int(t[1])
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')])
    a = np.memmap(path, dtype=dtype, mode='r', offset=off, shape=(n,))
    xyz = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)
    rgb_i = a['rgb'].view(np.uint32)
    rgb = np.stack([(rgb_i & 255), ((rgb_i >> 8) & 255), ((rgb_i >> 16) & 255)],
                   axis=1).astype(np.float64)
    ok = np.isfinite(xyz).all(axis=1)
    return xyz[ok], rgb[ok]

VS = 0.02
def voxel_stats(xyz, rgb):
    key = np.floor(xyz / VS).astype(np.int64)
    key = key - key.min(0)
    dims = key.max(0) + 1
    idx = (key[:, 0] * dims[1] + key[:, 1]) * dims[2] + key[:, 2]
    uniq, inv, cnt = np.unique(idx, return_inverse=True, return_counts=True)
    mu = np.zeros((len(uniq), 3))
    rms = np.zeros(len(uniq))
    for c in range(3):
        s = np.bincount(inv, weights=rgb[:, c], minlength=len(uniq))
        s2 = np.bincount(inv, weights=rgb[:, c] ** 2, minlength=len(uniq))
        mu[:, c] = s / cnt
        rms += np.maximum(s2 / cnt - (s / cnt) ** 2, 0)
    return uniq, cnt, mu, np.sqrt(rms / 3.0)

res = {}
for k, p in PATHS.items():
    xyz, rgb = load_pcd(p)
    res[k] = voxel_stats(xyz, rgb)
    print(k, 'voxels', len(res[k][0]))

# 用 A 的体素键去匹配（键是同一坐标系下的绝对格号，但各云 min 不同 -> 用世界格号重算）
def abs_keys(xyz):
    key = np.floor(xyz / VS).astype(np.int64)
    return (key[:, 0] * 73856093) ^ (key[:, 1] * 19349663) ^ (key[:, 2] * 83492791)

# 重新加载只算键（内存已释放）
stats = {}
for k, p in PATHS.items():
    xyz, rgb = load_pcd(p)
    key = abs_keys(xyz)
    order = np.argsort(key, kind='stable')
    key_s = key[order]
    xyz_s, rgb_s = xyz[order], rgb[order]
    uniq, start, cnt = np.unique(key_s, return_index=True, return_counts=True)
    mu = np.zeros((len(uniq), 3)); rms = np.zeros(len(uniq))
    for c in range(3):
        csum = np.add.reduceat(rgb_s[:, c], start)
        cs2 = np.add.reduceat(rgb_s[:, c] ** 2, start)
        mu[:, c] = csum / cnt
        rms += np.maximum(cs2 / cnt - (csum / cnt) ** 2, 0)
    rms = np.sqrt(rms / 3.0)
    maxidx = np.maximum.reduceat(np.arange(len(key_s)), start)
    centers = xyz_s[maxidx]
    stats[k] = dict(key=uniq, cnt=cnt, mu=mu, rms=rms, ctr=centers)
    del xyz, rgb, key, order, key_s, xyz_s, rgb_s

m = {}
for k in stats:
    keep = stats[k]['cnt'] >= 5
    m[k] = {kk: vv[keep] for kk, vv in stats[k].items()}
    print(k, 'voxels>=5pts', keep.sum(), 'colorRMS mean', m[k]['rms'].mean().round(2))

common, ia, ib = np.intersect1d(m['A']['key'], m['B']['key'], return_indices=True)
common2, ia2, io = np.intersect1d(common, m['OFF']['key'], return_indices=True)
ia = ia[ia2]; ib = ib[ia2]
rA, rB, rO = m['A']['rms'][ia], m['B']['rms'][ib], m['OFF']['rms'][io]
ctr = m['A']['ctr'][ia]
print(f'\n三云共有的 >=5 点体素: {len(common2)}')
print(f'  local color RMS  mean: A={rA.mean():.2f} B={rB.mean():.2f} OFF={rO.mean():.2f}')
print(f'  p99:                   A={np.percentile(rA,99):.1f} B={np.percentile(rB,99):.1f} OFF={np.percentile(rO,99):.1f}')
thr = np.percentile(rA, 99)
sel = rA >= thr
print(f'\nA 最差的 1% 体素（RMS>={thr:.1f}）共 {sel.sum()} 个:')
print(f'  A={rA[sel].mean():.1f}  B={rB[sel].mean():.1f}  OFF={rO[sel].mean():.1f}')
print(f'  这些体素在 B 中仍 >{thr:.0f} 的比例: {(rB[sel]>=thr).mean()*100:.1f}%  在 OFF 中: {(rO[sel]>=thr).mean()*100:.1f}%')
# 差异最大区域（A-OFF 平均色差）
dmean = np.abs(m['A']['mu'][ia] - m['OFF']['mu'][io]).mean(axis=1)
top = np.argsort(dmean)[-10:]
print('\nA 与 OFF 平均色差最大的 10 个体素（中心 xyz）:')
for i in top:
    print(f'  {ctr[i].round(2)}  dRGB={dmean[i]:.0f}  RMS A={rA[i]:.1f} B={rB[i]:.1f} OFF={rO[i]:.1f}')
