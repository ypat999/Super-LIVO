#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""全量场景按帧染色：读 Super-LIO 逐帧导出的 scans_*.pcd（世界系稠密点），
每帧一个随机高饱和色，合并导出整场景 PCD + 俯视全密度渲染图 + ANOVA 分解。

与 frame_id_color_map.py 的区别：那个吃 40 帧小样本 CSV；这个吃整场跑（~1300 帧、
千万级点）的逐帧文件，帧号 = 文件序号 = 时间序。

用法：
  python3 tools/frame_color_from_scans.py \
      --src /home/ywj/slam_data/pcd/retail_framecolor/PCD \
      --out /home/ywj/slam_data/pcd/retail_framecolor/color
"""
import argparse
import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame_id_color_map import write_pcd, frame_colors, anova

HDR_RE = re.compile(r"^(VERSION|FIELDS|SIZE|TYPE|COUNT|WIDTH|HEIGHT|VIEWPOINT|POINTS|DATA)\b")


def read_pcd_xyz(path):
    """读 binary PCD，只取 x y z（float32）。"""
    with open(path, "rb") as f:
        fields, sizes, counts, n = None, None, None, 0
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            if line.startswith("#"):
                continue
            if not HDR_RE.match(line):
                raise ValueError(f"bad header at {line!r}")
            tk = line.split()
            if tk[0] == "FIELDS":
                fields = tk[1:]
            elif tk[0] == "SIZE":
                sizes = [int(v) for v in tk[1:]]
            elif tk[0] == "TYPE":
                types = tk[1:]
            elif tk[0] == "COUNT":
                counts = [int(v) for v in tk[1:]]
            elif tk[0] == "POINTS":
                n = int(tk[1])
            elif tk[0] == "DATA":
                assert tk[1] == "binary", f"{path}: only binary PCD supported"
                break
        dt_map = {("F", 4): "f4", ("F", 8): "f8", ("U", 1): "u1", ("U", 2): "u2",
                  ("U", 4): "u4", ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4"}
        names, fmt, off = [], [], 0
        for nm, sz, tp, ct in zip(fields, sizes, types, counts):
            for k in range(ct):
                names.append(f"{nm}_{k}" if ct > 1 else nm)
                fmt.append(dt_map[(tp, sz)])
                off += sz
        raw = np.frombuffer(f.read(n * off), dtype=np.dtype([(nm, ft) for nm, ft in zip(names, fmt)]))
    return np.stack([raw["x"], raw["y"], raw["z"]], 1).astype(np.float32)


def raster_top(P, fr, C, px=4000):
    """全密度俯视栅格：后画帧覆盖先画帧（时间序=覆盖序，可看最后写入层的错位）。"""
    xmin, ymin = P[:, 0].min(), P[:, 1].min()
    span = max(P[:, 0].max() - xmin, P[:, 1].max() - ymin, 1e-6)
    ix = ((P[:, 0] - xmin) / span * (px - 1)).astype(np.int64)
    iy = ((P[:, 1] - ymin) / span * (px - 1)).astype(np.int64)
    img = np.full((px, px, 3), 255, np.uint8)
    img[px - 1 - iy, ix] = C[fr]          # 按点序逐次覆盖（numpy 花式赋值同像素取后写）
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="/home/ywj/slam_data/pcd/retail_framecolor/PCD")
    ap.add_argument("--out", default="/home/ywj/slam_data/pcd/retail_framecolor/color")
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    files = sorted((f for f in os.listdir(a.src) if re.fullmatch(r"scans_\d+\.pcd", f)),
                   key=lambda s: int(s[6:-4]))
    print(f"{len(files)} 帧文件")
    P, fr = [], []
    for i, fn in enumerate(files):
        p = read_pcd_xyz(os.path.join(a.src, fn))
        P.append(p)
        fr.append(np.full(len(p), i, np.int32))
        if (i + 1) % 200 == 0:
            print(f"  loaded {i+1}/{len(files)}")
    P = np.concatenate(P)
    fr = np.concatenate(fr)
    nf = len(files)
    print(f"总点数 {len(P)}")

    stats = {"n_points": int(len(P)), "n_frames": nf}
    legend = {}
    for mode, fname in (("random", "frame_random_full.pcd"), ("hue", "frame_hue_full.pcd")):
        C = frame_colors(nf, a.seed, mode)
        write_pcd(os.path.join(a.out, fname), P, C[fr])
        if mode == "random":
            legend = {str(i): [int(C[i, 0]), int(C[i, 1]), int(C[i, 2]),
                               int((fr == i).sum())] for i in range(nf)}
            with open(os.path.join(a.out, "frame_legend.json"), "w") as f:
                json.dump(legend, f)
            img = raster_top(P, fr, C)
            try:
                from matplotlib.image import imsave
                imsave(os.path.join(a.out, "preview_random_full.png"), img)
                print("preview_random_full.png 已输出（全密度俯视）")
            except Exception as e:
                print(f"PNG 跳过：{e}")

    for e in (0.02, 0.05):
        s = anova(P.astype(np.float64), fr.astype(np.int64), e)
        stats[f"anova_{int(e*100)}cm"] = s
        if s.get("n_voxels"):
            print(f"[{int(e*100)}cm] 体素数={s['n_voxels']}  "
                  f"跨帧std中位={s['between_std_cm']['med']:.2f}cm "
                  f"帧内std中位={s['within_std_cm']['med']:.2f}cm "
                  f"重影极差中位={s['ghost_range_cm']['med']:.2f}cm "
                  f"跨帧方差占比中位={s['between_var_frac_med']:.2f}")
    with open(os.path.join(a.out, "summary.json"), "w") as f:
        json.dump(stats, f, indent=1)


if __name__ == "__main__":
    main()
