#!/usr/bin/env python3
"""对比 Retail 上方案 A / 方案 B 的轨迹与彩色点云（同一 LIO 主干、同一份 bag）。

用法: python3 _cmp_ab.py [A目录] [B目录]
"""
import os
import sys

import numpy as np


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    return d


def summarize(tag, d):
    t = (d["t_ns"] - d["t_ns"][0]) * 1e-9
    xyz = np.stack([d["x"], d["y"], d["z"]], axis=1)
    step = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    spd = np.linalg.norm(np.stack([d["vx"], d["vy"], d["vz"]], axis=1), axis=1)
    print("%-3s rows=%4d  t=[0,%.2f]s  start=(%.3f %.3f %.3f)  end=(%.3f %.3f %.3f)"
          % (tag, len(t), t[-1], *xyz[0], *xyz[-1]))
    print("    路径长=%.2fm  位移=%.2fm  平均|v|=%.2fm/s  最大|v|=%.2fm/s"
          % (step.sum(), np.linalg.norm(xyz[-1] - xyz[0]), spd.mean(), spd.max()))
    return t, xyz


def pcd_info(path):
    if not os.path.exists(path):
        return "（无文件）"
    n = None
    with open(path, "rb") as f:
        for _ in range(12):
            line = f.readline()
            if line.startswith(b"POINTS"):
                n = int(line.split()[1])
                break
    return "%d 点, %.1f MB" % (n or -1, os.path.getsize(path) / 1e6)


def main():
    base = "/home/ywj/slam_data/output/retail_ab_1114"
    a_dir = sys.argv[1] if len(sys.argv) > 1 else base + "/a"
    b_dir = sys.argv[2] if len(sys.argv) > 2 else base + "/b"

    da = load(os.path.join(a_dir, "odom.csv"))
    db = load(os.path.join(b_dir, "odom.csv"))
    ta, xa = summarize("A", da)
    tb, xb = summarize("B", db)

    # 以 A 的时间轴为准做最近邻配对（|dt|<=20 ms）
    idx = np.searchsorted(tb, ta)
    idx = np.clip(idx, 1, len(tb) - 1)
    left = np.abs(tb[idx - 1] - ta)
    right = np.abs(tb[idx] - ta)
    pick = np.where(right < left, idx, idx - 1)
    ok = np.minimum(left, right) <= 0.02
    dt = xa[ok] - xb[pick[ok]]
    dist = np.linalg.norm(dt, axis=1)
    print("\nA/B 配对 %d 帧：位置差 中位=%.3f m  p95=%.3f m  末端=%.3f m"
          % (ok.sum(), np.median(dist), np.percentile(dist, 95), dist[-1]))

    for tag, d in (("A", a_dir), ("B", b_dir)):
        p = [f for f in os.listdir(d) if f.endswith(".pcd")] if os.path.isdir(d) else []
        for f in sorted(p):
            print("%-3s 彩色点云 %s: %s" % (tag, f, pcd_info(os.path.join(d, f))))


if __name__ == "__main__":
    main()
