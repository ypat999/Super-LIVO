#!/usr/bin/env python3
"""椒盐（颜色离群）比例评估。

指标定义（不要用体内 RGB std：每体素只有 1 个点时 std 恒为 0，没有意义）：
  每点颜色 vs 其 3 cm 邻域（27 格，按体素中位色）中位色的逐通道最大绝对偏差，
  统计偏差 >40 / >60 / >80（0-255）的占比。

基线：旧 append 1.98%@>60，官方 FAST-LIVO2 1.68%@>60。
"""
import argparse
import sys

import numpy as np

VS = 0.03  # 邻域体素边长（米）
SHIFT = 1 << 20
MASK = (1 << 21) - 1


def read_pcd(path):
    with open(path, "rb") as f:
        fields = None
        count = None
        data_start = 0
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError("PCD 头部异常")
            text = line.decode("ascii", "ignore").strip()
            if text.startswith("FIELDS"):
                fields = text.split()[1:]
            elif text.startswith("POINTS"):
                count = int(text.split()[1])
            elif text.startswith("DATA"):
                data_start = f.tell()
                break
    dtype = np.dtype([(name, "<f4") for name in fields])
    raw = np.fromfile(path, dtype=dtype, count=count, offset=data_start)
    xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float64)
    # rgb 以 float32 位模式存储
    rgb_u32 = raw["rgb"].view(np.uint32)
    rgb = np.stack([(rgb_u32 >> 16) & 255, (rgb_u32 >> 8) & 255, rgb_u32 & 255], axis=1)
    return xyz, rgb.astype(np.float32)


def cell_keys(xyz, vs):
    idx = np.floor(xyz / vs).astype(np.int64)
    return (idx[:, 0] & MASK) | ((idx[:, 1] & MASK) << 21) | ((idx[:, 2] & MASK) << 42)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcd")
    ap.add_argument("--voxel", type=float, default=VS, help="邻域体素边长（米）")
    ap.add_argument("--sample", type=int, default=200000, help="采样点数（0=全量）")
    ap.add_argument("--dedupe", type=float, default=0.0,
                    help=">0 时先按该体素边长去重（保留最后一次观测），用于控制密度差异")
    args = ap.parse_args()

    xyz, rgb = read_pcd(args.pcd)
    if args.dedupe > 0.0:
        # 每个体素只保留最后一次观测（等价于 append 地图的"同格重复点"折叠）
        keys0 = cell_keys(xyz, args.dedupe)
        order = np.argsort(keys0, kind="stable")
        sk0 = keys0[order]
        keep = np.ones(len(order), dtype=bool)
        keep[:-1] = sk0[1:] != sk0[:-1]
        sel0 = order[keep]
        xyz, rgb = xyz[sel0], rgb[sel0]
    n = len(xyz)
    if n == 0:
        print("空点云")
        return
    keys = cell_keys(xyz, args.voxel)
    uniq, inv = np.unique(keys, return_inverse=True)
    m = len(uniq)

    # 逐体素逐通道中位数：按 (体素, 通道值) 排序后取组内中间元素
    med = np.zeros((m, 3), dtype=np.float32)
    for ch in range(3):
        order = np.lexsort((rgb[:, ch], inv))
        sorted_inv = inv[order]
        sorted_val = rgb[order, ch]
        starts = np.searchsorted(sorted_inv, np.arange(m), side="left")
        ends = np.searchsorted(sorted_inv, np.arange(m), side="right")
        mid = (starts + ends - 1) // 2
        med[:, ch] = sorted_val[mid]

    # 采样
    if args.sample and args.sample < n:
        sel = np.random.default_rng(0).choice(n, args.sample, replace=False)
    else:
        sel = np.arange(n)
    sk = keys[sel]

    # 27 邻域：在 uniq 上 searchsorted 查找
    offsets = [(dx & MASK) | ((dy & MASK) << 21) | ((dz & MASK) << 42)
               for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)]
    stack = np.empty((27, len(sel), 3), dtype=np.float32)
    stack[:] = np.nan
    for i, off in enumerate(offsets):
        # 加法需按 21 位分段进位安全：逐段拆开加
        bx = (sk & MASK) + (off & MASK)
        by = ((sk >> 21) & MASK) + ((off >> 21) & MASK)
        bz = ((sk >> 42) & MASK) + ((off >> 42) & MASK)
        nk = (bx & MASK) | ((by & MASK) << 21) | ((bz & MASK) << 42)
        pos = np.searchsorted(uniq, nk)
        pos = np.clip(pos, 0, m - 1)
        hit = uniq[pos] == nk
        stack[i, hit] = med[pos[hit]]

    nb_med = np.nanmedian(stack, axis=0)
    dev = np.max(np.abs(rgb[sel] - nb_med), axis=1)

    total = len(dev)
    print(f"文件: {args.pcd}")
    print(f"点数: {n:,}  体素: {m:,}（{args.voxel * 100:.0f}cm）  采样: {total:,}")
    for th in (40, 60, 80):
        print(f"  >{th}: {np.mean(dev > th) * 100:.2f}%")


if __name__ == "__main__":
    sys.exit(main())
