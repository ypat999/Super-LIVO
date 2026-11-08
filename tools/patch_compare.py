#!/usr/bin/env python3
"""同一区域裁剪点云的正面对比：密度、法向分层、鬼影、边缘锐度，并输出渲染图。

两张裁剪图覆盖完全相同的物理区域，所以密度可比；再叠加一个
"沿主平面法线做 z-buffer"的正交渲染，直接复现肉眼盯着招牌看的效果。

用法:
  python3 patch_compare.py <a.pcd> <b.pcd> [--px=0.002] [--png=DIR]
"""
import os
import sys
import numpy as np
from scipy.ndimage import median_filter, minimum_filter, maximum_filter


def read_pcd(path):
    with open(path, "rb") as f:
        h = {}
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            if not line:
                continue
            ps = line.split()
            h[ps[0].upper()] = ps[1:]
            if ps[0].upper() == "DATA":
                data = ps[1].lower()
                break
        n = int(h["POINTS"][0])
        if data != "binary":
            raise SystemExit("只支持 binary: " + path)
        fields = h["FIELDS"]
        sizes = [int(s) for s in h["SIZE"]]
        types = h["TYPE"]
        counts = ([int(c) for c in h["COUNT"]] if "COUNT" in h
                  else [1] * len(fields))
        strides = [sz * cn for sz, cn in zip(sizes, counts)]
        buf = np.frombuffer(f.read(n * sum(strides)), dtype=np.uint8).reshape(n, sum(strides))
    off, pos = {}, 0
    for name, sz, ty, st in zip(fields, sizes, types, strides):
        off[name] = (pos, sz, ty)
        pos += st

    def col(name):
        p, sz, ty = off[name]
        return buf[:, p:p + sz].copy()

    xyz = np.stack([col("x").view(np.float32).ravel(),
                    col("y").view(np.float32).ravel(),
                    col("z").view(np.float32).ravel()], 1)
    p, sz, ty = off["rgb"]
    if ty == "F":                       # PCL 把 RGB 塞进 float32 位模式，需按位重解释
        packed = col("rgb").view(np.uint32).ravel()
        rgb = np.stack([(packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF], 1
                       ).astype(np.uint8)
    else:
        rgb = col("rgb")[:, :3].astype(np.uint8)
    xyz = xyz.astype(np.float32)
    keep = np.isfinite(xyz).all(1) & (np.abs(xyz).max(1) < 1e4)
    if not keep.all():
        print(f"   [{path.split('/')[-1]}] 剔除无效点 {(~keep).sum():,}（{(~keep).mean()*100:.2f}%）")
    return xyz[keep], rgb[keep]


def fit_plane(P, iters=4, trim=0.05):
    """总最小二乘 + 迭代剔除，返回 (质心, 单位法向)。"""
    c = P.mean(0)
    n = None
    for _ in range(iters):
        Q = P - c
        if n is not None:
            d = np.abs(Q @ n)
            keep = d < np.percentile(d, 100 * (1 - trim))
            P2, c = P[keep], P[keep].mean(0)
        else:
            P2 = P
        Q = (P2 - c).astype(np.float64)
        u, s, vt = np.linalg.svd(Q, full_matrices=False)
        n = vt[2]
        if n[2] < 0:
            n = -n
    return c, n


def occupancy(xyz, cell):
    idx = np.floor((xyz - xyz.min(0)) / cell).astype(np.int64)
    nx = idx[:, 0].max() + 1
    ny = idx[:, 1].max() + 1
    _, inv = np.unique(idx[:, 0] + idx[:, 1] * nx + idx[:, 2] * nx * ny, return_inverse=True)
    m = int(inv.max()) + 1
    cnt = np.bincount(inv, minlength=m)
    return m, cnt, cnt[inv]


def main(paths, px=0.002, pngdir=""):
    data = []
    for p in paths:
        xyz, rgb = read_pcd(p)
        data.append((p, xyz, rgb))
    allp = np.concatenate([d[1] for d in data]).astype(np.float64)
    c, n = fit_plane(allp)
    # 面内正交基
    a1 = np.array([n[1], -n[0], 0], np.float64)
    a1 /= np.linalg.norm(a1) + 1e-12
    a2 = np.cross(n, a1)
    print(f"主平面: 质心 {np.round(c,3)}  法向 {np.round(n,3)}")

    for p, xyz, rgb in data:
        name = p.split("/")[-1].replace(".section.pcd", "")
        v = (xyz.astype(np.float64) - c)
        dn = v @ n
        u = v @ a1
        w = v @ a2
        print(f"\n== {name}  点 {len(xyz):,}")
        for cell in (0.01, 0.02, 0.05):
            m, cnt, pc = occupancy(xyz, cell)
            print(f"   {cell*100:2.0f}cm 格: 占用 {m:,}  每格中位 {np.median(cnt):.0f}  "
                  f"孤立点(格内1) {np.mean(pc==1)*100:5.2f}%  "
                  f"稀疏(<=2) {np.mean(pc<=2)*100:5.2f}%  密集(>=8) {np.mean(pc>=8)*100:5.2f}%")
        ad = np.abs(dn)
        print(f"   主平面法向偏离 cm: 中位 {np.median(ad)*100:5.2f}  P75 {np.percentile(ad,75)*100:5.2f}  "
              f"P90 {np.percentile(ad,90)*100:5.2f}  P99 {np.percentile(ad,99)*100:5.2f} | "
              f">2cm {np.mean(ad>0.02)*100:5.2f}%  >4cm {np.mean(ad>0.04)*100:5.2f}%  "
              f">8cm {np.mean(ad>0.08)*100:5.2f}%")
        # 有符号法向分布 -> 分层鬼影（多张偏移的"皮"）
        h, e = np.histogram(dn, bins=np.arange(-0.15, 0.151, 0.005))
        prof = " ".join(f"{int(x):>5}" for x in h[::2])
        print(f"   有符号法向直方图(-15..15cm, 每1cm): {prof}")
        # 面内包围盒外的浮点：切向范围
        print(f"   面内范围: u {np.percentile(u,[1,99])*100} cm  w {np.percentile(w,[1,99])*100} cm  "
              f"面积(2cm格) {occupancy(xyz,0.02)[0]*4} cm^2")

    # 正交渲染对比
    if pngdir:
        os.makedirs(pngdir, exist_ok=True)
        import struct
        for p, xyz, rgb in data:
            v = xyz.astype(np.float64) - c
            uu, wv, dn = v @ a1, v @ a2, v @ n
            iu = np.floor((uu - uu.min()) / px).astype(np.int64)
            iw = np.floor((wv - wv.min()) / px).astype(np.int64)
            W_, H_ = int(iu.max()) + 1, int(iw.max()) + 1
            pix = iw * W_ + iu
            zb = np.full(W_ * H_, np.inf, np.float64)
            np.minimum.at(zb, pix, dn)
            flat = np.zeros((W_ * H_, 3), np.uint8)
            order = np.argsort(-dn)
            flat[pix[order]] = rgb[order]
            img = flat.reshape(H_, W_, 3)
            filled = np.isfinite(zb).reshape(H_, W_)
            stray = (dn - zb[pix])
            # 空洞填充：LiDAR 稀疏，不填的话椒盐/边缘指标会被背景主导
            from scipy.ndimage import distance_transform_edt
            dist, nearest = distance_transform_edt(~filled, return_indices=True)
            img = img[nearest[0], nearest[1]]
            lum = (0.299 * img[:, :, 0] + 0.587 * img[:, :, 1]
                   + 0.114 * img[:, :, 2]).astype(np.float32)
            lmin = minimum_filter(lum, 9, mode="nearest")
            lmax = maximum_filter(lum, 9, mode="nearest")
            con = lmax - lmin
            em = (con > 60) & filled
            t01 = (lum[em] - lmin[em]) / con[em]
            med = median_filter(lum, 5, mode="nearest")
            spk = np.mean((np.abs(lum - med) > 40)[filled])
            print(f"\n   [渲染 {W_}x{H_}] {p.split('/')[-1]}")
            print(f"      填充率 {filled.mean()*100:.1f}%  皮后鬼影 >2cm {np.mean(stray>0.02)*100:5.2f}%  "
                  f">4cm {np.mean(stray>0.04)*100:5.2f}%")
            print(f"      边缘中间调占比(模糊度) {np.mean((t01>0.2)&(t01<0.8))*100:5.2f}%  "
                  f"椒盐像素 {spk*100:5.2f}%")
            stem = p.split("/")[-1].replace(".section.pcd", "")
            try:
                from PIL import Image
                Image.fromarray(img).save(f"{pngdir}/{stem}.png")
                print(f"      -> {pngdir}/{stem}.png  ({img.shape[1]}x{img.shape[0]})")
            except Exception as ex:
                print(f"      PNG 写出失败: {ex}")


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    px, pngdir = 0.002, ""
    for x in sys.argv[1:]:
        if x.startswith("--px="):
            px = float(x.split("=")[1])
        if x.startswith("--png="):
            pngdir = x.split("=")[1]
    main(a, px, pngdir)
