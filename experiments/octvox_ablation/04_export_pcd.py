#!/usr/bin/env python3
"""04：导出 A/B/C/D 代表点为二进制 PCD（x y z + count + src_frame）。

PCD 字段：x y z (float32, 世界系, m)，count (uint16, 子体素累计观测数)，
src_frame (int32, 代表点最后来源帧的帧序号，-1 表示 D 组无单一来源)。
D 组每点单独导出；A/B/C 每子体素 1 点。

输出目录：/home/ywj/slam_data/octvox_ablation_20261110/pcd/
"""
import os
import numpy as np

OUT = "/home/ywj/slam_data/octvox_ablation_20261110"
DST = os.path.join(OUT, "pcd")

FIELDS = [("x", "F", 4), ("y", "F", 4), ("z", "F", 4),
          ("count", "U", 2), ("src_frame", "I", 4)]


def write_pcd(path, pts, cnt, src):
    n = len(pts)
    header = ("# .PCD v0.7 - Point Cloud Data file format\n"
              "VERSION 0.7\n"
              f"FIELDS {' '.join(f for f, _, _ in FIELDS)}\n"
              f"SIZE {' '.join(str(s) for _, _, s in FIELDS)}\n"
              f"TYPE {' '.join(t for _, t, _ in FIELDS)}\n"
              f"COUNT {' '.join('1' for _ in FIELDS)}\n"
              f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {n}\nDATA binary\n")
    dt = np.dtype([(f, "<f4" if t == "F" else ("<u2" if s == 2 else "<i4"))
                   for f, t, s in FIELDS])
    arr = np.empty(n, dtype=dt)
    arr["x"], arr["y"], arr["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    arr["count"] = np.clip(cnt, 0, 65535).astype(np.uint16)
    arr["src_frame"] = src.astype(np.int32)
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(arr.tobytes())
    print(f"{os.path.basename(path)}: {n} pts")


def main():
    os.makedirs(DST, exist_ok=True)
    for mode in ("A", "B", "C"):
        d = np.load(os.path.join(OUT, f"map_{mode}.npz"))
        write_pcd(os.path.join(DST, f"map_{mode}.pcd"),
                  d["points"].astype(np.float32), d["count"], d["src_frame"])
    d = np.load(os.path.join(OUT, "map_D.npz"))
    write_pcd(os.path.join(DST, "map_D.pcd"),
              d["points"].astype(np.float32), d["count"], d["src_frame"])


if __name__ == "__main__":
    main()
