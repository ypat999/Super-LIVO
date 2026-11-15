#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""色纹理对比：体素内色标准差。

动机：5cm 体素平均色 MAE 只有 5.91，已低于参考自劈半的 7.95，但肉眼观感
仍是官方更"实"。怀疑差异不在平均色，而在纹理被抹平：
  1cm 体素时间中位回填会把同体素内所有点写成同一个色 → 高频纹理消失、成片平色。
官方 publish_frame_world 不做任何融合，逐帧逐点保留原始采样 → 纹理完整。

指标：对每个 VOX 体素算 RGB 三通道标准差，再按点数加权求全图均值；
      同时给出"平色体素"(std<2) 占比。纹理越丰富，intra-std 越大。

用法: python3 _probe_color_texture.py <pcd1> [pcd2 ...]
"""
import sys
import numpy as np

VOX_LIST = (0.01, 0.05)
QN = 6000000


def header(path):
    with open(path, "rb") as f:
        txt = b""
        while not txt.endswith(b"DATA binary\n"):
            c = f.read(1)
            if not c:
                break
            txt += c
    lines = txt.decode("ascii", "ignore").splitlines()
    flds = [l.split()[1:] for l in lines if l.startswith("FIELDS")][0]
    n = int([l.split()[1] for l in lines if l.startswith("POINTS")][0])
    return flds, n, len(txt)


def load(path):
    f, n, off = header(path)
    dt = np.dtype({"names": f, "formats": ["<f4"] * len(f),
                   "offsets": [4 * i for i in range(len(f))], "itemsize": 4 * len(f)})
    arr = np.memmap(path, dtype=dt, mode="r", offset=off, shape=(n,))
    if n > QN:  # 大图下采样，保证内存与耗时可控
        idx = np.sort(np.random.default_rng(0).choice(n, QN, replace=False))
        arr = arr[idx]
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float32)
    key = "rgb" if "rgb" in arr.dtype.names else "rgba"
    v = np.asarray(arr[key]).view(np.uint32)
    rgb = np.stack([(v >> 16) & 255, (v >> 8) & 255, v & 255], 1).astype(np.float32)
    return xyz, rgb


def texture(path):
    xyz, rgb = load(path)
    print("\n== %s ==  %d 点(抽样)" % (path.split("/")[-1], len(xyz)))
    for vox in VOX_LIST:
        q = np.floor(xyz / vox).astype(np.int64) + (1 << 20)
        k = (q[:, 0] << 42) | (q[:, 1] << 21) | q[:, 2]
        _, inv, cnt = np.unique(k, return_inverse=True, return_counts=True)
        del k, q
        # 逐通道方差 = E[c^2] - E[c]^2，再取三通道平均
        var = np.zeros(len(cnt), dtype=np.float64)
        for ch in range(3):
            s1 = np.bincount(inv, weights=rgb[:, ch], minlength=len(cnt))
            s2 = np.bincount(inv, weights=rgb[:, ch] ** 2, minlength=len(cnt))
            m = s1 / cnt
            var += np.maximum(s2 / cnt - m * m, 0.0)
        var /= 3.0
        std = np.sqrt(var)
        multi = cnt >= 2
        w = cnt[multi].astype(np.float64)
        intra = float((std[multi] * w).sum() / w.sum()) if multi.any() else 0.0
        print("  %4.0fcm  体素 %8d  平均点数 %5.2f  多点数体素 %5.1f%%  "
              "体素内色std(按点加权) %5.2f  平色体素(std<2) %5.1f%%" % (
                  vox * 100, len(cnt), cnt.mean(), 100.0 * multi.mean(),
                  intra, 100.0 * (std[multi] < 2).mean()))


if __name__ == "__main__":
    for p in sys.argv[1:]:
        texture(p)
    print("\nDONE_COLOR_TEXTURE")
