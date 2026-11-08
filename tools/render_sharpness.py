#!/usr/bin/env python3
"""把点云"渲染成图"来量化肉眼看到的模糊/鬼影 —— 密度无关。

前面几个指标（16-NN 厚度、邻域外推残差、可用率）都栽在同一件事上：
它们都要先在点云里"找邻居"，而两张图的点密度不同（官方 88.8 点/10cm 格，
SL 64.5），密度本身就会改变邻居距离，于是指标测到的其实是"稀不稀"，
不是"糊不糊"。

本脚本改用**沿射线定义表面**，与密度无关：
  1. 取官方轨迹上若干位姿当虚拟相机，把点投影到像平面；
  2. 每个像素取**最近**深度做 z-buffer —— 这条"最近表面"就是眼睛看到的皮；
  3. 落在皮后面/前面的点 = 鬼影点，统计其占比（stray）；
  4. 在 z-buffer 彩色图上量两个肉眼指标：
     - 边缘过渡像素占比（模糊度）：强对比邻域内，亮度处于 20%-80% 之间的
       像素比例。边界越锐利，中间调像素越少；重影会把边界摊开成灰带。
     - 椒盐率：与 5x5 中位色偏差 >40 的像素占比（染色噪声）。

用法:
  python3 render_sharpness.py <poses.txt> <a.pcd> [b.pcd ...] [--views=12] [--out=DIR]
"""
import os
import sys
import numpy as np
from scipy.ndimage import median_filter, minimum_filter, maximum_filter

W, H = 376, 240          # 半分辨率渲染，够快且与肉眼判断一致
FX = FY = 460.0
CX, CY = 376.0, 240.0


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
    rgb = arr[:, 12:].copy().view(np.uint32).reshape(n)
    r = ((rgb >> 16) & 0xFF).astype(np.uint8)
    g = ((rgb >> 8) & 0xFF).astype(np.uint8)
    b = (rgb & 0xFF).astype(np.uint8)
    return xyz, np.stack([r, g, b], 1)


def read_poses(path):
    a = np.loadtxt(path, usecols=(0, 1, 2, 3, 4, 5, 6, 7))
    return a                                        # t x y z qx qy qz qw


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]],
        dtype=np.float32)


def render(xyz, rgb, poses, nviews, rng, nsub=3_000_000):
    idx = rng.choice(len(xyz), min(nsub, len(xyz)), replace=False)
    P, C = xyz[idx], rgb[idx]
    # IMU/LiDAR 系 (x 前, y 左, z 上) -> 光学系 (z 前, x 右, y 下)
    M = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], np.float32)
    sel = np.linspace(0, len(poses) - 1, nviews).astype(int)
    stray3 = stray6 = total = 0
    mid_frac = speck = edgepix = filledpix = 0
    imgs = []
    for vi, pi in enumerate(sel):
        t = poses[pi, 1:4].astype(np.float32)
        R = quat_to_R(poses[pi, 4:8])
        pc = (P - t) @ (R @ M.T)                     # world -> 光学系
        z = pc[:, 2]
        ok = (z > 1.0) & (z < 30.0)
        u = np.floor(FX * pc[ok, 0] / z[ok] + CX).astype(np.int32)
        v = np.floor(FY * pc[ok, 1] / z[ok] + CY).astype(np.int32)
        zz = z[ok]
        cc = C[ok]
        inb = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        u, v, zz, cc = u[inb], v[inb], zz[inb], cc[inb]
        pix = v * W + u
        total += len(zz)
        # z-buffer：每个像素取最小深度
        zbuf = np.full(W * H, np.inf, np.float32)
        np.minimum.at(zbuf, pix, zz)
        d = zz - zbuf[pix]
        stray3 += int((d > 0.03).sum())
        stray6 += int((d > 0.06).sum())
        # 取每个像素最近点的颜色（降序写入，最近的最后写 -> 覆盖前面）
        order = np.argsort(-d)
        img = np.zeros((H, W, 3), np.uint8) + 30
        flat = np.zeros((H * W, 3), np.uint8)
        flat[pix[order]] = cc[order]
        img = flat.reshape(H, W, 3)
        lum = (0.299 * img[:, :, 0] + 0.587 * img[:, :, 1]
               + 0.114 * img[:, :, 2]).astype(np.float32)
        filled = np.isfinite(zbuf).reshape(H, W)
        if filled.mean() < 0.15:
            continue
        lmin = minimum_filter(lum, 9, mode="nearest")
        lmax = maximum_filter(lum, 9, mode="nearest")
        contrast = lmax - lmin
        em = (contrast > 60) & filled
        if em.sum() > 200:
            t01 = (lum[em] - lmin[em]) / contrast[em]
            mid_frac += int(((t01 > 0.2) & (t01 < 0.8)).sum())
            edgepix += int(em.sum())
        med = median_filter(lum, 5, mode="nearest")
        speck += int((np.abs(lum - med) > 40)[filled].sum())
        filledpix += int(filled.sum())
        if vi in (0, nviews // 2):
            imgs.append(img.copy())
    return dict(stray3=stray3 / total, stray6=stray6 / total,
                mid=mid_frac / max(edgepix, 1), speck=speck / max(filledpix, 1),
                views=len(imgs), imgs=imgs)


def main(pose_path, paths, nviews=12, out=""):
    poses = read_poses(pose_path)
    print(f"轨迹 {len(poses)} 位姿，取 {nviews} 个视角，渲染 {W}x{H}")
    for p in paths:
        xyz, rgb = read_pcd(p)
        rng = np.random.default_rng(0)               # 同一随机种子 -> 同一采样比例
        r = render(xyz, rgb, poses, nviews, rng)
        print(f"\n  {p.rsplit('/',2)[-2]:<30} 点 {len(xyz):,}")
        print(f"      鬼影点(在可见皮后面) >3cm {r['stray3']*100:5.2f}%   "
              f">6cm {r['stray6']*100:5.2f}%")
        print(f"      边缘中间调占比(模糊度)   {r['mid']*100:5.2f}%")
        print(f"      椒盐像素(偏离5x5中位>40) {r['speck']*100:5.2f}%")
        if out:
            os.makedirs(out, exist_ok=True)
            import struct
            for i, img in enumerate(r["imgs"]):
                name = f"{out}/{p.rsplit('/',2)[-2]}_v{i}.npy"
                np.save(name, img)
        del xyz, rgb


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    nviews, out = 12, ""
    for x in sys.argv[1:]:
        if x.startswith("--views="):
            nviews = int(x.split("=")[1])
        if x.startswith("--out="):
            out = x.split("=")[1]
    main(a[0], a[1:], nviews, out)
