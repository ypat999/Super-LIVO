#!/usr/bin/env python3
"""点云"鬼影/毛躁"体检（全分辨率 + 邻域外推平面）。

为什么之前的指标测不出肉眼看到的差异：
  1) 随机抽样 25 万点做 16-NN 局部平面 —— 抽样后 NN 距离被放大 ~4 倍，
     参考平面本身跨了 10 cm 以上，正好把 5-15 cm 的鬼影"吸收"进平面；
  2) 局部平面是自拟合的：鬼影层自己也是一张平面，拟合时会把平面拉向
     自己，残差自然小；
  3) 取全图中位数 —— 被大面积平整墙面主导，而眼睛看的是笔画/边界这类
     细高对比结构。

本脚本的三点改动：
  A. 不抽样，全部点参与统计；
  B. 参考平面"邻域外推"：先把点聚成 cell 取质心，拟合某 cell 的平面时
     只用它周围的 cell 质心、**排除自己**，再拿该 cell 内的全部点算残差。
     鬼影层无法把平面拉向自己，偏离会被如实计为残差；
  C. 按 cell 内亮度 std 分成 edge（笔画/边界）与 flat（大平面）两组分别统计。

用法:
  python3 pointcloud_ghost_eval.py a.pcd [b.pcd ...] [--cell=0.10] [--k=8]
"""
import sys
import numpy as np
from scipy.spatial import cKDTree


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
    lum = (0.299 * ((rgb >> 16) & 0xFF) + 0.587 * ((rgb >> 8) & 0xFF)
           + 0.114 * (rgb & 0xFF)).astype(np.float32)
    return xyz, lum


def evaluate(path, cell=0.10, k=8, min_cnt=1):
    xyz, lum = read_pcd(path)
    ok = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).max(axis=1) < 5000)
    xyz, lum = xyz[ok], lum[ok]
    n = len(xyz)

    idx = np.floor((xyz - xyz.min(axis=0)) / cell).astype(np.int64)
    nx = idx[:, 0].max() + 1
    ny = idx[:, 1].max() + 1
    key = idx[:, 0] + idx[:, 1] * nx + idx[:, 2] * nx * ny
    _, inv = np.unique(key, return_inverse=True)
    m = int(inv.max()) + 1

    cnt = np.bincount(inv, minlength=m).astype(np.float64)
    cen = np.stack([np.bincount(inv, weights=xyz[:, i].astype(np.float64), minlength=m)
                    for i in range(3)], 1) / cnt[:, None]
    lmean = (np.bincount(inv, weights=lum, minlength=m) / cnt).astype(np.float32)
    lstd = np.sqrt(np.clip(np.bincount(inv, weights=lum * lum, minlength=m) / cnt
                           - lmean ** 2, 0, None)).astype(np.float32)

    # 邻域外推平面：self 距离为 0 必排在第 0 列，取其后的 k 个质心拟合
    _, ii = cKDTree(cen).query(cen, k=k + 1)
    P = cen[ii[:, 1:]].astype(np.float32)               # (m, k, 3)
    del cen, ii
    mu = P.mean(axis=1)
    Q = P - mu[:, None]
    del P
    cov = np.einsum("cki,ckj->cij", Q, Q) / k
    w, v = np.linalg.eigh(cov)
    del cov, w
    nrm = np.ascontiguousarray(v[:, :, 0])              # 最小特征值方向 = 法向
    del v
    nb_dev = np.abs(np.einsum("cki,ci->ck", Q, nrm))
    del Q
    planar = nb_dev.max(axis=1) < 0.5 * cell            # 邻域本身接近平面才可用
    del nb_dev
    usable = planar & (cnt >= min_cnt)

    d = np.abs(((xyz - mu[inv].astype(np.float32)) * nrm[inv]).sum(axis=1))
    m_ok = usable[inv]
    edge = lstd[inv] > 18.0

    def blk(mask, tag):
        if mask.sum() < 1000:
            return
        v = d[mask]
        print(f"     {tag:<6} 占点 {mask.mean()*100:5.1f}%  "
              f"中位 {np.median(v)*100:5.2f}  P75 {np.percentile(v,75)*100:5.2f}  "
              f"P90 {np.percentile(v,90)*100:5.2f}  P99 {np.percentile(v,99)*100:5.2f} cm |"
              f" >3cm {np.mean(v>0.03)*100:5.2f}%  >6cm {np.mean(v>0.06)*100:5.2f}%  "
              f">10cm {np.mean(v>0.10)*100:5.2f}%")

    print(f"\n== {path.rsplit('/',2)[-2]}  点 {n:,}  cell {cell*100:.0f}cm k={k}")
    print(f"   cell {m:,}  邻域外推可用 {m_ok.mean()*100:.1f}%  "
          f"edge(高对比) 占点 {edge.mean()*100:.1f}%  "
          f"颜色椒盐>40 {np.mean(np.abs(lum-lmean[inv])>40)*100:.2f}%")
    blk(m_ok, "全部")
    blk(m_ok & ~edge, "flat")
    blk(m_ok & edge, "edge")

    # 密度分层：同格点数相近时才可比，否则比的是"覆盖率"而不是"几何质量"
    cc = cnt[inv]
    print("   -- 按 10cm 格内点数分层（同密度才可比）--")
    print(f"      {'格内点数':<10}{'占点%':>8}{'格数占比%':>11}{'残差中位cm':>11}"
          f"{'P90cm':>8}{'>3cm%':>8}{'>6cm%':>8}{'可用%':>8}")
    for lo, hi in [(1, 2), (3, 8), (9, 20), (21, 50), (51, 10 ** 9)]:
        mm = (cc >= lo) & (cc <= hi)
        if mm.sum() < 1000:
            continue
        v = d[mm & m_ok]
        tag = f"{lo}-{hi if hi < 10**8 else 'inf'}"
        if v.size < 1000:
            print(f"      {tag:<10}{mm.mean()*100:>8.1f}{((cnt>=lo)&(cnt<=hi)).mean()*100:>11.1f}"
                  f"{'—':>11}{'—':>8}{'—':>8}{'—':>8}{m_ok[mm].mean()*100:>8.1f}")
            continue
        print(f"      {tag:<10}{mm.mean()*100:>8.1f}{((cnt>=lo)&(cnt<=hi)).mean()*100:>11.1f}"
              f"{np.median(v)*100:>11.2f}{np.percentile(v,90)*100:>8.2f}"
              f"{np.mean(v>0.03)*100:>8.2f}{np.mean(v>0.06)*100:>8.2f}"
              f"{m_ok[mm].mean()*100:>8.1f}")
    print(f"      每格点数: 均值 {n/m:.1f}  中位 {np.median(cnt):.0f}  "
          f"cnt<=2 的格占 {np.mean(cnt<=2)*100:.1f}%（这些格贡献 "
          f"{np.mean(cc<=2)*100:.1f}% 的点）")

    # 绝对网格下的"孤立度"：与密度无关，直接对应肉眼看到的散点/浮点
    print("   -- 孤立度（绝对网格，格内只有 1-2 个点 = 浮点）--")
    for c2 in (0.02, 0.05, 0.10):
        i2 = np.floor((xyz - xyz.min(axis=0)) / c2).astype(np.int64)
        a_, b_ = i2[:, 0].max() + 1, i2[:, 1].max() + 1
        _, inv2 = np.unique(i2[:, 0] + i2[:, 1] * a_ + i2[:, 2] * a_ * b_,
                            return_inverse=True)
        n2 = int(inv2.max()) + 1
        c2cnt = np.bincount(inv2, minlength=n2)
        pc = c2cnt[inv2]
        print(f"      {c2*100:4.0f}cm 网格: 格数 {n2:,}  每格中位 {np.median(c2cnt):.0f} 点  "
              f"孤立点(格内仅1) {np.mean(pc==1)*100:5.2f}%  "
              f"稀疏点(格内<=2) {np.mean(pc<=2)*100:5.2f}%  "
              f"密集体(格内>=8) {np.mean(pc>=8)*100:5.2f}%")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    cell, k = 0.10, 8
    for a in sys.argv[1:]:
        if a.startswith("--cell="):
            cell = float(a.split("=")[1])
        if a.startswith("--k="):
            k = int(a.split("=")[1])
    for p in args:
        evaluate(p, cell, k)
