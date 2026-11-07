#!/usr/bin/env python3
"""彩色点云跨帧一致性评测。

判据：把世界系彩色点云按体素(默认 1cm)分组，统计同一体素内 RGB 的标准差并按体素平均。
标准差越小 = 同一物理点在不同帧被染的颜色越一致 = 着色/位姿对齐越好。
比单纯 MAE 对"椒盐错位"敏感得多，是本项目定位时间基准 bug 的决定性指标。

用法: color_consistency.py [--voxel 0.01] a.pcd b.pcd ...
依赖: 仅 numpy（直接解析 PCD binary，无需 open3d/pcl）。
"""
import argparse
import numpy as np


def load_pcd_rgb(path):
    """解析 binary PCD（FIELDS 含 x y z rgb），返回 xyz(N,3) float32、rgb(N,3) 顺序 R,G,B。"""
    with open(path, 'rb') as f:
        hdr = {}
        while True:
            line = f.readline().decode('ascii', 'replace').strip()
            p = line.split()
            if not p:
                continue
            hdr[p[0]] = p[1:]
            if p[0] == 'DATA':
                assert p[1:] == ['binary'], '仅支持 binary PCD'
                break
        n = int(hdr['POINTS'][0])
        fields, sizes = hdr['FIELDS'], [int(x) for x in hdr['SIZE']]
        step = sum(sizes)
        raw = np.frombuffer(f.read(n * step), np.uint8).reshape(n, step)
    off = {}
    acc = 0
    for fn, sz in zip(fields, sizes):
        off[fn] = (acc, sz)
        acc += sz
    xyz = np.stack([raw[:, off[k][0]:off[k][0] + 4].copy().view(np.float32).ravel()
                    for k in ('x', 'y', 'z')], 1)
    # PCD 的 rgb 字段是打包 uint32：R 在高字节。
    rgb = np.stack([raw[:, off['rgb'][0] + i].astype(np.float32) for i in (2, 1, 0)], 1)
    return xyz, rgb


def voxel_color_std(xyz, rgb, voxel):
    ix = np.floor(xyz[:, 0] / voxel).astype(np.int64) + 200000
    iy = np.floor(xyz[:, 1] / voxel).astype(np.int64) + 200000
    iz = np.floor(xyz[:, 2] / voxel).astype(np.int64) + 200000
    key = (ix * 400001 + iy) * 400001 + iz
    _, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    std = []
    for ch in range(3):
        m = np.bincount(inv, weights=rgb[:, ch], minlength=len(cnt)) / cnt
        m2 = np.bincount(inv, weights=rgb[:, ch] ** 2, minlength=len(cnt)) / cnt
        std.append(float(np.sqrt(np.maximum(m2 - m ** 2, 0)).mean()))
    return std


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcds', nargs='+')
    ap.add_argument('--voxel', type=float, default=0.01)
    args = ap.parse_args()
    for path in args.pcds:
        xyz, rgb = load_pcd_rgb(path)
        # 只统计真正上过色的点（非纯黑），避免未着色点稀释。
        mask = rgb.sum(1) > 0
        std = voxel_color_std(xyz[mask], rgb[mask], args.voxel)
        print('%-34s 点=%d(有色%d)  %.0fmm体素颜色std R=%.1f G=%.1f B=%.1f'
              % (path.rsplit('/', 1)[-1], len(xyz), int(mask.sum()), args.voxel * 1000,
                 std[0], std[1], std[2]))


if __name__ == '__main__':
    main()
