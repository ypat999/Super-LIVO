#!/usr/bin/env python3
"""以官方地图为"干净参考面"，量化另一张地图的点相对参考面的偏离，
并把偏离拆成 **法向**（沿平面法线）与 **切向**（在平面内滑动）两部分。

为什么这个拆法能定位原因：
  - 只有传感器测距噪声 / 帧内去畸变误差  -> 偏离几乎全在 **法向**；
  - 帧间位姿抖动（多帧各贴一张偏移的"皮"） -> 偏离同时出现在 **切向**，
    表现为边界重影、笔画变粗、面板外扩出一圈浮点。
肉眼看到的"鬼影/边界不清"对应切向项，"表面粗糙"对应法向项。

用法: python3 ghost_vs_ref.py <ref.pcd> <test.pcd> [<test2.pcd> ...] [--cell=0.10]
"""
import sys
import numpy as np
from scipy.spatial import cKDTree


def read_pcd(path):
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            if not line:
                continue
            parts = line.split()
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                fmt = parts[1].lower()
                break
        n = int(header["POINTS"][0])
        if fmt != "binary":
            raise SystemExit("只支持 binary pcd: " + path)
        buf = f.read()
    arr = np.frombuffer(buf[: n * 16], dtype=np.uint8).reshape(n, 16)
    xyz = arr[:, :12].copy().view(np.float32).reshape(n, 3)
    return xyz


def cells(xyz, cell):
    idx = np.floor((xyz - xyz.min(axis=0)) / cell).astype(np.int64)
    nx = idx[:, 0].max() + 1
    ny = idx[:, 1].max() + 1
    _, inv = np.unique(idx[:, 0] + idx[:, 1] * nx + idx[:, 2] * nx * ny,
                       return_inverse=True)
    m = int(inv.max()) + 1
    cnt = np.bincount(inv, minlength=m).astype(np.float64)
    cen = np.stack([np.bincount(inv, weights=xyz[:, i].astype(np.float64), minlength=m)
                    for i in range(3)], 1) / cnt[:, None]
    return inv.astype(np.int32), cnt, cen


def planes(cen, cnt, cell, k=8, min_cnt=20):
    """邻域外推平面：排除自身质心，用周围 k 个质心拟合，要求邻域本身接近平面。"""
    _, ii = cKDTree(cen).query(cen, k=k + 1)
    P = cen[ii[:, 1:]].astype(np.float32)
    mu = P.mean(axis=1)
    Q = P - mu[:, None]
    cov = np.einsum("cki,ckj->cij", Q, Q) / k
    w, v = np.linalg.eigh(cov)
    nrm = np.ascontiguousarray(v[:, :, 0])
    span = np.abs(np.einsum("cki,ci->ck", Q, nrm)).max(axis=1)
    ok = (span < 0.4 * cell) & (cnt >= min_cnt)
    return mu, nrm, ok


def main(ref_path, test_paths, cell=0.10):
    r = read_pcd(ref_path)
    r = r[np.isfinite(r).all(axis=1)]
    _, rcen_cnt, rcen = cells(r, cell)
    mu, nrm, ok = planes(rcen, rcen_cnt, cell)
    sel = np.where(ok)[0]
    mu_s, nrm_s = mu[sel], nrm[sel]
    del r, mu, nrm, rcen_cnt
    print(f"参考面 {len(sel):,} 个（{cell*100:.0f}cm 格、邻域外推、格内>=20 点）")
    tree = cKDTree(mu_s)

    for p in test_paths:
        t = read_pcd(p)
        t = t[np.isfinite(t).all(axis=1)]
        dd, jj = tree.query(t, k=1)
        near = dd < 1.5 * cell                       # 只统计参考面覆盖到的地方
        nom = (t[near] - mu_s[jj[near]]).astype(np.float32)
        nr = nrm_s[jj[near]]
        dn = np.abs(np.einsum("ni,ni->n", nom, nr))
        dt = np.linalg.norm(nom - dn[:, None] * nr, axis=1)
        del dd, jj, nom, nr
        name = p.rsplit("/", 2)[-2]
        print(f"\n  {name:<32} 落在参考面附近 {near.mean()*100:5.1f}%  "
              f"(n={near.sum():,})")
        for tag, a in (("法向", dn), ("切向", dt)):
            print(f"      {tag}偏离 cm: 中位 {np.median(a)*100:5.2f}  "
                  f"P75 {np.percentile(a,75)*100:5.2f}  P90 {np.percentile(a,90)*100:5.2f}  "
                  f"P99 {np.percentile(a,99)*100:5.2f} | "
                  f">2cm {np.mean(a>0.02)*100:5.2f}%  >4cm {np.mean(a>0.04)*100:5.2f}%  "
                  f">8cm {np.mean(a>0.08)*100:5.2f}%")
        # 切向/法向比：>1 说明偏离主要是"滑动"而非"测距噪声"
        print(f"      切向/法向 P90 比 = {np.percentile(dt,90)/max(np.percentile(dn,90),1e-6):.2f}")
        del t


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    cell = 0.10
    for x in sys.argv[1:]:
        if x.startswith("--cell="):
            cell = float(x.split("=")[1])
    main(a[0], a[1:], cell)
