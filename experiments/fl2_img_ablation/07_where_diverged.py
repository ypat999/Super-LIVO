#!/usr/bin/env python3
"""Retail 无图像 m0r2 整轮发散的时间分布：跳变是否集中在起步收敛期。

用法：python3 07_where_diverged.py
"""
import numpy as np

D = "/home/ywj/slam_data/fl2_img_abl_20261110/retail"
for tag in ("m0r1", "m0r2", "m0r3", "m1r1"):
    a = np.loadtxt(f"{D}/{tag}/pose.txt")
    t, p = a[:, 0], a[:, 1:4]
    t0 = t[0]
    step = np.linalg.norm(np.diff(p, axis=0), axis=1)
    bad = np.where(step > 0.3)[0]
    print(f"\n{tag}: 跳变帧 {len(bad)} / {len(a)}")
    if len(bad):
        hist = np.histogram(t[bad] - t0, bins=np.arange(0, 140, 10))[0]
        print("  跳变按 10 s 分箱: " + " ".join(f"{b:.0f}-{b+10:.0f}s:{h}"
                                                for b, h in zip(np.arange(0, 140, 10), hist) if h))
        print(f"  首个跳变 t=+{t[bad[0]]-t0:.1f}s，最后 t=+{t[bad[-1]]-t0:.1f}s")
        cum = np.cumsum(step)
        print(f"  0-30s 路径 {cum[300]:.1f} m，30-135s 路径 {cum[-1]-cum[300]:.1f} m"
              f"（同期有图像轮 0-30s≈13 m）")
