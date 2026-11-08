#!/usr/bin/env python3
"""点云"毛刺/偏离点"体检：按到最近轨迹位姿的距离分箱，统计局部厚度与离群点占比。
用法: python3 rough_eval.py <map.pcd> <poses.txt> [sample]
"""
import sys
import numpy as np
from scipy.spatial import cKDTree


def read_xyz(path):
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            if not line:
                continue
            parts = line.split()
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                break
        n = int(header["POINTS"][0])
        data = np.frombuffer(f.read(), dtype=np.float32)
    step = data.size // n
    return data.reshape(n, step)[:, :3].astype(np.float64)


def read_poses(path):
    p = np.loadtxt(path, usecols=(1, 2, 3))
    return p


def main(pcd, posef, sample=250_000):
    xyz = read_xyz(pcd)
    n_all = len(xyz)
    rng = np.random.default_rng(0)
    idx = rng.choice(n_all, min(sample, n_all), replace=False)
    s = xyz[idx]

    # 到最近轨迹位姿的距离 = 传感器量程（地图在 world 系）
    poses = read_poses(posef)
    ptree = cKDTree(poses)
    rng_m, _ = ptree.query(s, k=1)

    tree = cKDTree(s)
    d, nn = tree.query(s, k=17)
    d1 = d[:, 1]
    neigh = s[nn[:, 1:]]
    neigh = neigh - neigh.mean(axis=1, keepdims=True)
    cov = np.einsum("mki,mkj->mij", neigh, neigh) / 16
    ev = np.linalg.eigvalsh(cov)
    thick = np.sqrt(np.clip(ev[:, 0], 0, None))
    # 点到局部平面的距离（法向量由最小特征向量给出）
    _, vecs = np.linalg.eigh(cov)
    nrm = vecs[:, :, 0]
    neigh_mean = s[nn[:, 1:]].mean(axis=1)
    dist = np.abs(np.einsum("mk,mk->m", s - neigh_mean, nrm))

    bins = [(0, 5), (5, 10), (10, 15), (15, 20), (20, 30), (30, 50), (50, 1e9)]
    print(f"\n== {pcd.split('/')[-2]}  总点数 {n_all:,}  采样 {len(s):,}")
    print(f"   全局: NN中位 {np.median(d1)*100:.2f} cm | 厚度中位 {np.median(thick)*100:.2f} cm"
          f" | 厚度P90 {np.percentile(thick,90)*100:.2f} cm"
          f" | 离面>5cm {np.mean(dist>0.05)*100:.2f}% | >10cm {np.mean(dist>0.10)*100:.2f}%")
    for cut in (15, 20, 30):
        m = rng_m < cut
        print(f"   若裁剪 <{cut}m: 保留 {m.mean()*100:.1f}% 点 | 厚度中位 {np.median(thick[m])*100:.2f} cm"
              f" | 离面>5cm {np.mean(dist[m]>0.05)*100:.2f}% | >10cm {np.mean(dist[m]>0.10)*100:.2f}%")
    print(f"   {'量程(m)':<12}{'占比%':>8}{'NN中位cm':>10}{'厚度中位cm':>11}{'厚度P90cm':>10}{'离面>5cm%':>10}{'离面>10cm%':>11}")
    for lo, hi in bins:
        m = (rng_m >= lo) & (rng_m < hi)
        if m.sum() < 200:
            continue
        hi_s = "inf" if hi > 1e8 else str(hi)
        print(f"   {f'{lo}-{hi_s}':<12}{m.mean()*100:>8.1f}"
              f"{np.median(d1[m])*100:>10.2f}{np.median(thick[m])*100:>11.2f}"
              f"{np.percentile(thick[m],90)*100:>10.2f}"
              f"{np.mean(dist[m]>0.05)*100:>10.2f}{np.mean(dist[m]>0.10)*100:>11.2f}")


if __name__ == "__main__":
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 250_000
    main(sys.argv[1], sys.argv[2], n)
