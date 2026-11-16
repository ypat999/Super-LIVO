#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分层（ghosting / layering）量化探针。

动机：肉眼看"墙面重影"是分层，但体素统计只能说明"弥散"。本脚本给两个
可直接比较的分层量：

A. 局部平面厚度：每个点取 k 近邻拟合平面，算该点到平面的法向距离。
   单帧 LiDAR 打在同一面墙上的点厚度 ~2-3 cm；同一墙面被搬到位移不同的
   多个位置 → 邻域跨层 → 法向距离被拉到分米级。P50/P90 就是"层厚"。

B. 全局平面多峰：RANSAC 找最大的几个平面，取平面芯区 ±H 内的点，
   沿法向做 1 cm 直方图，找显著峰。峰数 >1 且峰间距 ≈ Δ 上限 ⇒ 直接坐实分层，
   且间距随信任域上限缩放就是归因成立的判决性证据。

用法: python3 _probe_layering.py <pcd1> [pcd2 ...]
"""
import sys
import numpy as np
from scipy.spatial import cKDTree

SUB = 300000      # KNN 子样点数（各臂同种子，规模可比）
K = 16            # 近邻数
SEED = 0
PLANE_H = 0.60    # B 项：沿法向的搜索半宽（m）
BIN = 0.01        # 法向直方图 bin（m）


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
    """按 FIELDS 解析：本产物是 x y z rgb（4×float32），只取前三列做几何。"""
    flds, n, off = header(path)
    dt = np.dtype({"names": flds, "formats": ["<f4"] * len(flds),
                   "offsets": [4 * i for i in range(len(flds))],
                   "itemsize": 4 * len(flds)})
    arr = np.memmap(path, dtype=dt, mode="r", offset=off, shape=(n,))
    rng = np.random.default_rng(SEED)
    idx = rng.choice(n, min(SUB, n), replace=False) if n > SUB else np.arange(n)
    p = np.stack([arr["x"][idx], arr["y"][idx], arr["z"][idx]],
                 1).astype(np.float64)
    ok = np.isfinite(p).all(1) & (np.abs(p).max(1) < 1e4)   # 剔除未初始化大值
    return p[ok]


def local_thickness(p):
    """A 项：每点到其 k 近邻拟合平面的法向距离。"""
    tree = cKDTree(p)
    _, ii = tree.query(p, k=K)
    nb = p[ii]                              # (N,K,3)
    c = nb.mean(1)
    q = nb - c[:, None, :]
    cov = np.einsum("nki,nkj->nij", q, q) / K
    w, v = np.linalg.eigh(cov)                          # 最小特征值对应法向
    nrm = v[:, :, 0]
    d = np.abs(np.einsum("ni,ni->n", p - c, nrm))
    return d


def peaks(h, min_rel=0.12, min_sep=4):
    """1 cm 直方图上的显著峰：高度 > min_rel×max，间距 >= min_sep 个 bin。"""
    if h.size < 3:
        return []
    k = np.ones(3) / 3.0
    s = np.convolve(h, k, mode="same")
    m = s.max()
    if m <= 0:
        return []
    cand = [i for i in range(1, s.size - 1)
            if s[i] >= s[i - 1] and s[i] >= s[i + 1] and s[i] >= min_rel * m]
    cand.sort(key=lambda i: -s[i])
    keep = []
    for i in cand:
        if all(abs(i - j) >= min_sep for j in keep):
            keep.append(i)
    return sorted(keep)


def global_planes(p, want=4):
    """B 项：迭代 RANSAC 找最大的几个平面并做多峰分析。"""
    rng = np.random.default_rng(SEED + 1)
    work = p
    out = []
    for _ in range(60):
        if len(work) < 50000 or len(out) >= want:
            break
        sub = work[rng.choice(len(work), min(150000, len(work)), replace=False)]
        best = (0, None)
        for _t in range(160):
            a, b, c = sub[rng.choice(len(sub), 3, replace=False)]
            n = np.cross(b - a, c - a)
            nn = np.linalg.norm(n)
            if nn < 1e-6:
                continue
            n /= nn
            d = np.abs((sub - a) @ n)
            inl = int((d < 0.05).sum())
            if inl > best[0]:
                best = (inl, (n, a, d))
        inl, got = best
        if got is None or inl < 0.03 * len(work):
            break
        n, a, _ = got
        d_all = (work - a) @ n
        band = np.abs(d_all) < PLANE_H
        core = work[band]
        if len(core) < 5000:
            work = work[np.abs(d_all) > 0.05]
            continue
        # 芯区：平面内包围盒收缩到 5-95 分位，避开边缘斜坡
        u = np.random.default_rng(7).standard_normal((3, 2))
        u -= n[:, None] @ (n[None, :] @ u)
        u, _ = np.linalg.qr(u)
        s2 = core @ u
        lo, hi = np.percentile(s2, 5, axis=0), np.percentile(s2, 95, axis=0)
        sel = ((s2 >= lo) & (s2 <= hi)).all(1)
        dd = (core[sel] - a) @ n
        if len(dd) < 3000:
            work = work[np.abs(d_all) > 0.05]
            continue
        edges = np.arange(-PLANE_H, PLANE_H + BIN / 2, BIN)
        h, _ = np.histogram(dd, bins=edges)
        pk = peaks(h.astype(float))
        pos = [(edges[i] + BIN / 2) * 100 for i in pk]
        sp = [round(pos[i + 1] - pos[i], 1) for i in range(len(pos) - 1)]
        out.append(dict(nrm=np.round(n, 2).tolist(), pts=int(len(dd)),
                        thick_p50=float(np.percentile(np.abs(dd), 50) * 1000),
                        thick_p95=float(np.percentile(np.abs(dd), 95) * 1000),
                        peaks=len(pk), peak_cm=[round(x, 1) for x in pos],
                        sep_cm=sp))
        work = work[np.abs(d_all) > 0.05]
    return out


def probe(path):
    p = load(path)
    print("\n== %s ==  %d 点(抽样)" % (path.split("/")[-1], len(p)))
    d = local_thickness(p)
    print("  A 局部平面厚度(mm)  P50=%6.2f  P75=%6.2f  P90=%6.2f  P99=%7.2f  >10cm %4.1f%%" % (
        np.percentile(d, 50) * 1000, np.percentile(d, 75) * 1000,
        np.percentile(d, 90) * 1000, np.percentile(d, 99) * 1000,
        100.0 * (d > 0.10).mean()))
    for i, pl in enumerate(global_planes(p)):
        print("  B 平面%d 法向%s 点数%-7d 厚度P50=%5.1f P95=%6.1f mm  "
              "峰数=%d 峰位(cm)%s 间距(cm)%s" % (
                  i, pl["nrm"], pl["pts"], pl["thick_p50"], pl["thick_p95"],
                  pl["peaks"], pl["peak_cm"], pl["sep_cm"]))


if __name__ == "__main__":
    for x in sys.argv[1:]:
        probe(x)
    print("\nDONE_PROBE_LAYERING")
