#!/usr/bin/env python3
"""点云质量指标集筛选实验（从 gpt_点云指标集.md 的 100+ 候选里挑出 32 个）。

实验设计（对应文档第四节"如何判断哪个指标有效"）：
  1. 相同 ROI：A/B 用同一物理区域（用户从两张图截出的招牌裁剪）。
  2. 分块配对：把区域沿面内 u 轴切 8 列、w 轴切 2 行 = 16 个 patch，
     两幅图按同位置配对 -> 每个指标有 16 个成对样本，可算效应量与一致性。
  3. 区分能力：d = mean(A-B)/|mean|（相对差）+ 一致性（16 对里有多少对同向）。
  4. 参数鲁棒：局部几何族在 3 个物理半径（10/20/40mm）各算一遍，看排序会不会翻。
  5. 冗余度：所有指标在 32 个 patch 取值上的 Spearman 相关，|rho|>0.95 视为冗余。

用法:
  python3 qc_metric_screen.py <A.pcd> <B.pcd> [--cols=8] [--rows=2] [--px=0.002]
"""
import sys
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import median_filter, maximum_filter, distance_transform_edt, map_coordinates

EPS = 1e-12


# ---------------------------------------------------------------- 数据读取
def read_pcd(path):
    with open(path, "rb") as f:
        h = {}
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            if not line:
                continue
            ps = line.split()
            h[ps[0].upper()] = ps[1:]
            if ps[0].upper() == "DATA":
                assert ps[1].lower() == "binary", path
                break
        n = int(h["POINTS"][0])
        sizes = [int(s) for s in h["SIZE"]]
        counts = [int(c) for c in h["COUNT"]] if "COUNT" in h else [1] * len(sizes)
        types = h["TYPE"]
        strides = [s * c for s, c in zip(sizes, counts)]
        buf = np.frombuffer(f.read(n * sum(strides)), np.uint8).reshape(n, sum(strides))
    off, pos = {}, 0
    for name, sz, ty, st in zip(h["FIELDS"], sizes, types, strides):
        off[name] = (pos, sz, ty)
        pos += st

    def col(name):
        p, sz, _ = off[name]
        return buf[:, p:p + sz].copy()

    xyz = np.stack([col("x").view(np.float32).ravel(),
                    col("y").view(np.float32).ravel(),
                    col("z").view(np.float32).ravel()], 1).astype(np.float64)
    p, sz, ty = off["rgb"]
    if ty == "F":
        pk = col("rgb").view(np.uint32).ravel()
        rgb = np.stack([(pk >> 16) & 0xFF, (pk >> 8) & 0xFF, pk & 0xFF], 1).astype(np.uint8)
    else:
        rgb = col("rgb")[:, :3]
    keep = np.isfinite(xyz).all(1)
    return xyz[keep], rgb[keep]


def rgb2lab(rgb):
    c = rgb.astype(np.float64) / 255.0
    c = np.where(c > 0.04045, ((c + 0.055) / 1.055) ** 2.4, c / 12.92)
    M = np.array([[0.4124, 0.3576, 0.1805], [0.2126, 0.7152, 0.0722],
                  [0.0193, 0.1192, 0.9505]])
    X, Y, Z = (c @ M.T)[:, 0], (c @ M.T)[:, 1], (c @ M.T)[:, 2]
    X /= 0.95047
    Z /= 1.08883
    def g(t):
        return np.where(t > 0.008856, np.cbrt(t), 7.787 * t + 16 / 116)
    fx, fy, fz = g(X), g(Y), g(Z)
    return np.stack([116 * fy - 16, 500 * (fx - fy), 200 * (fy - fz)], 1)


def lum(rgb):
    return 0.2126 * rgb[:, 0] + 0.7152 * rgb[:, 1] + 0.0722 * rgb[:, 2]


# ---------------------------------------------------------------- 局部几何
def local_moments(P, K):
    """固定邻居数 K 的一阶/二阶矩（比固定半径内存可控，且天然多尺度）。
    返回 (mask, 邻域均值, 特征值(n,3), 法向(n,3), 实际邻域半径均值)。"""
    n = len(P)
    tree = cKDTree(P)
    dd, ii = tree.query(P, k=K)
    Pn = P[ii]                                          # (n,K,3)
    mean = Pn.mean(1)
    Q = Pn - mean[:, None, :]
    C = np.einsum("nki,nkj->nij", Q, Q) / K
    ev, vec = np.linalg.eigh(C)
    lam = np.clip(ev, 0, None)
    nrm = vec[:, :, 0].copy()
    sg = np.sign(np.einsum("ni,ni->n", nrm, P - mean))
    nrm *= (sg == 0)[:, None] + sg[:, None]
    d = np.ones(n, bool)
    return d, mean, lam, nrm, dd[:, -1]


# ---------------------------------------------------------------- 指标计算
def metrics(P, rgb, ref_plane, px, K):
    """P: (n,3) patch 点; rgb: (n,3) uint8; ref_plane=(c,nrm); K: 邻域邻居数"""
    out = {}
    n = len(P)
    c0, n0 = ref_plane
    dn = (P - c0) @ n0
    # --- G 局部几何
    d, mean, lam, nrm, radii = local_moments(P, K)
    rough = np.sqrt(lam[:, 2])
    sv = lam[:, 2] / np.maximum(lam.sum(1), EPS)
    out["G00_Nb_Radius_mm"] = float(np.mean(radii)) * 1000
    out["G07_PCA_Roughness"] = float(np.mean(rough))
    out["G03_Plane_MAD"] = float(np.median(np.abs(dn)))
    out["G05_Plane_P95"] = float(np.percentile(np.abs(dn), 95))
    out["G08_Surface_Variation"] = float(np.mean(sv))
    out["G15_Roughness_CV"] = float(np.std(rough) / max(np.mean(rough), EPS))
    # 法向一致性：点法向与其最近邻法向夹角的 RMS
    t2 = cKDTree(nrm)
    _, j = t2.query(nrm, k=2)
    a = nrm[j[:, 0]], nrm[j[:, 1]]
    cosang = np.clip(np.abs(np.einsum("ni,ni->n", *a)), 0, 1)
    out["G12_Normal_Dispersion"] = float(np.sqrt(np.mean(np.arccos(cosang) ** 2)))
    lp = np.abs((P - mean) * nrm).sum(1)
    out["G10_Local_Residual_Std"] = float(np.std(lp))
    out["G11_Local_Residual_P95"] = float(np.percentile(np.abs(lp), 95))
    # --- D 分布
    nnd, nni = cKDTree(P).query(P, k=2)
    dd = nnd
    nn = dd[:, 1]
    out["D01_Mean_NN"] = float(np.mean(nn))
    out["D02_Median_NN"] = float(np.median(nn))
    out["D04_NN_CV"] = float(np.std(nn) / max(np.mean(nn), EPS))
    out["D05_NN_P95"] = float(np.percentile(nn, 95))
    for tag, cell in (("1cm", 0.01), ("2cm", 0.02)):
        idx = np.floor((P - P.min(0)) / cell).astype(np.int64)
        nx = idx[:, 0].max() + 1
        ny = idx[:, 1].max() + 1
        _, inv = np.unique(idx[:, 0] + idx[:, 1] * nx + idx[:, 2] * nx * ny, return_inverse=True)
        m = int(inv.max()) + 1
        cnt = np.bincount(inv, minlength=m)
        pc = cnt[inv]
        p_ = cnt / cnt.sum()
        out[f"D08_Density_CV_{tag}"] = float(np.std(cnt) / max(np.mean(cnt), EPS))
        out[f"D09_Density_P5_{tag}"] = float(np.percentile(cnt, 5))
        out[f"D12_Occupancy_Entropy_{tag}"] = float(-(p_ * np.log(p_ + EPS)).sum() / np.log(m))
        if tag == "1cm":
            out["D11_Empty_Cell_Ratio"] = float(np.mean(cnt == 0))
            out["O08_Isolated_Ratio_1cm"] = float(np.mean(pc == 1))
    # --- O 离群
    mu = np.mean(nn)
    out["O01_SOR_Ratio"] = float(np.mean(nn > mu + 2.0 * np.std(nn)))
    tree = cKDTree(P)
    nb3 = np.fromiter((len(x) for x in tree.query_ball_point(P, 0.03)), np.int32, n)
    out["O02_ROR_Ratio_3cm"] = float(np.mean(nb3 < 6))
    out["O05_Radius_Isolation_5cm"] = float(np.mean(
        np.fromiter((len(x) for x in tree.query_ball_point(P, 0.05)), np.int32, n) < 10))
    # --- C 颜色（邻域色差）
    lab = rgb2lab(rgb)
    pl = lab[nni[:, 1]] - lab
    de = np.sqrt((pl ** 2).sum(1))
    out["C03_dE76_Mean"] = float(np.mean(de))
    out["C04_dE76_P95"] = float(np.percentile(de, 95))
    # --- M 与参考面的关系（O10 / M10 的替代：相对主平面的法向偏离）
    out["O10_Off_Plane_Ratio_4cm"] = float(np.mean(np.abs(dn) > 0.04))
    out["M10_Normal_Dist_Mean"] = float(np.mean(np.abs(lp)))
    # --- 2D 正交栅格：E 边缘族 + C10/C11/C13/C15/C16
    a1 = np.array([n0[1], -n0[0], 0.0]); a1 /= np.linalg.norm(a1) + EPS
    a2 = np.cross(n0, a1)
    v = P - c0
    u_, w_ = v @ a1, v @ a2
    iu = np.floor((u_ - u_.min()) / px).astype(np.int64)
    iw = np.floor((w_ - w_.min()) / px).astype(np.int64)
    W_, H_ = int(iu.max()) + 1, int(iw.max()) + 1
    if W_ * H_ < 400 or H_ < 8:
        return out
    pix = iw * W_ + iu
    zb = np.full(W_ * H_, np.inf)
    np.minimum.at(zb, pix, dn)
    filled = np.isfinite(zb).reshape(H_, W_)
    if filled.mean() < 0.3:
        return out
    L = np.zeros((H_, W_))
    order = np.argsort(-dn)
    flat = np.zeros(H_ * W_)
    flat[pix[order]] = lum(rgb)[order]
    Lg = flat.reshape(H_, W_)
    _, near = distance_transform_edt(~filled, return_indices=True)
    Lg = Lg[near[0], near[1]]
    gy, gx = np.gradient(Lg, px)
    gm = np.hypot(gx, gy)
    gmax = maximum_filter(gm, 7)
    em = (gm >= 0.5 * gmax) & (gm > np.percentile(gm, 90)) & filled
    out["E04_Edge_Gradient"] = float(np.mean(gm[em])) if em.sum() else np.nan
    out["E11_Projected_Sharpness"] = float(np.mean(gm[filled] ** 2))
    out["E07_Edge_Completeness"] = float(em.sum() / max(filled.sum(), 1))
    # ESF：沿梯度方向采样 9 点，归一化后取 10-90 宽度
    if em.sum() >= 20:
        yy, xx = np.where(em)
        sel = np.random.default_rng(0).choice(len(yy), min(400, len(yy)), replace=False)
        ny_, nx_ = gy[yy[sel], xx[sel]], gx[yy[sel], xx[sel]]
        ln = np.hypot(ny_, nx_) + EPS
        dy, dx = -ny_ / ln, nx_ / ln
        t = np.arange(-4, 5)
        ys = yy[sel][:, None] + dy[:, None] * t[None, :]
        xs = xx[sel][:, None] + dx[:, None] * t[None, :]
        prof = map_coordinates(Lg, [ys.ravel(), xs.ravel()], order=1,
                               mode="nearest").reshape(len(sel), 9)
        lo = np.percentile(prof[:, :2], 50, axis=1)
        hi = np.percentile(prof[:, -2:], 50, axis=1)
        rng_ = hi - lo
        good = rng_ > 60
        if good.sum() >= 10:
            pn = (prof[good] - lo[good][:, None]) / rng_[good][:, None]
            def cross(pn, lvl):
                x = np.arange(9)
                res = []
                for r in pn:
                    r = np.clip(r, -0.2, 1.2)
                    i = np.argmax(r >= lvl) if (r >= lvl).any() else 8
                    if 0 < i < 9:
                        r0, r1 = r[i - 1], r[i]
                        f = 0 if abs(r1 - r0) < EPS else (lvl - r0) / (r1 - r0)
                        res.append(i - 1 + f)
                return np.median(res) if res else np.nan
            a90, a10 = cross(pn, 0.1), cross(pn, 0.9)
            b80, b20 = cross(pn, 0.2), cross(pn, 0.8)
            out["E01_Edge_Width_10_90_mm"] = abs(a90 - a10) * px * 1000
            out["E02_Edge_Width_20_80_mm"] = abs(b80 - b20) * px * 1000
            out["E12_ESF_Sigma_mm"] = float(np.std(np.gradient(pn, axis=1).mean(0))) * px * 1000
            out["E03_Edge_Position_Disp"] = float(np.std(pn.std(axis=1)))
    # CNR：亮度 Otsu 分前景/背景
    vals = Lg[filled]
    thr = float(np.percentile(vals, 50))
    for _ in range(20):
        f, b = vals[vals > thr], vals[vals <= thr]
        if f.size < 10 or b.size < 10:
            break
        thr = (f.mean() * b.std() + b.mean() * f.std()) / max(f.std() + b.std(), EPS)
    f, b = vals[vals > thr], vals[vals <= thr]
    if f.size > 10 and b.size > 10:
        out["C11_Luminance_CNR"] = float(abs(f.mean() - b.mean())
                                        / np.sqrt((f.var() + b.var()) / 2))
        labg = np.zeros((H_, W_, 3))
        for i in range(3):
            ch = np.zeros(H_ * W_)
            ch[pix[order]] = lab[order][:, i]
            labg[:, :, i] = ch.reshape(H_, W_)[near[0], near[1]]
        dE = np.sqrt(((labg - labg.mean()) ** 2).sum(2))
        out["C06_Local_Color_Variance"] = float(np.mean(dE[filled] ** 2))
    # Moran's I（亮度 4 邻域自相关）
    x = Lg - Lg[filled].mean()
    nbsum = (np.roll(x, 1, 0) + np.roll(x, -1, 0) + np.roll(x, 1, 1) + np.roll(x, -1, 1))[filled]
    nbf = np.full_like(x, 4.0)[filled]
    num = np.sum(x[filled] * nbsum / np.maximum(nbf, 1))
    den = np.sum(x[filled] ** 2)
    out["C15_Morans_I"] = float(num / max(den, EPS))
    # Variogram 块金比：lag-1 半方差 / 较大 lag 半方差中位数
    m1 = filled & np.roll(filled, 1, 1)
    d0 = (Lg - np.roll(Lg, 1, 1))[m1]
    far = []
    for lag in (8, 16, 24):
        mm = filled & np.roll(filled, lag, 1)
        if mm.sum() > 100:
            far.append(np.mean((Lg - np.roll(Lg, lag, 1))[mm] ** 2) / 2)
    out["C16_Variogram_Nugget_Ratio"] = (
        float(np.mean(d0 ** 2) / 2 / max(np.median(far), EPS)) if far else np.nan)
    return out


def cross_metrics(PA, PB, R):
    """M 族：两幅图同 patch 的互一致性。"""
    ta, tb = cKDTree(PA), cKDTree(PB)
    dab = tb.query(PA, k=1)[0]
    dba = ta.query(PB, k=1)[0]
    o = {}
    o["M01_Chamfer"] = float((dab.mean() + dba.mean()) / 2)
    o["M02_Chamfer_P95"] = float(np.percentile(np.concatenate([dab, dba]), 95))
    o["M03_Hausdorff"] = float(max(dab.max(), dba.max()))
    for t in (0.02, 0.05):
        pr = np.mean(dab < t)
        rc = np.mean(dba < t)
        o[f"M05_F_Score_{int(t*100)}cm"] = float(2 * pr * rc / max(pr + rc, EPS))
    return o


def main(pa, pb, cols=8, rows=2, px=0.002):
    A = read_pcd(pa)
    B = read_pcd(pb)
    allp = np.concatenate([A[0], B[0]])
    u, s, vt = np.linalg.svd(allp - allp.mean(0), full_matrices=False)
    c0, n0 = allp.mean(0), vt[2]
    if n0[2] < 0:
        n0 = -n0
    ref = (c0, n0)
    a1 = np.array([n0[1], -n0[0], 0.0]); a1 /= np.linalg.norm(a1) + EPS
    a2 = np.cross(n0, a1)

    def uvw(P):
        v = P - c0
        return v @ a1, v @ a2, v @ n0

    ua, wa, _ = uvw(A[0])
    ub, wb, _ = uvw(B[0])
    lo = min(ua.min(), ub.min())
    hi = max(ua.max(), ub.max())
    wlo = min(wa.min(), wb.min())
    whi = max(wa.max(), wb.max())
    print(f"ROI: u {lo:.2f}..{hi:.2f} m  w {wlo:.2f}..{whi:.2f} m  patch {cols}x{rows}")

    rowsout = []
    for ci in range(cols):
        for ri in range(rows):
            u0, u1 = lo + (hi - lo) * ci / cols, lo + (hi - lo) * (ci + 1) / cols
            w0, w1 = wlo + (whi - wlo) * ri / rows, wlo + (whi - wlo) * (ri + 1) / rows
            ma = (ua >= u0) & (ua < u1) & (wa >= w0) & (wa < w1)
            mb = (ub >= u0) & (ub < u1) & (wb >= w0) & (wb < w1)
            if ma.sum() < 3000 or mb.sum() < 3000:
                continue
            Pa, Ra = A[0][ma], A[1][ma]
            Pb, Rb = B[0][mb], B[1][mb]
            rec = {"patch": f"c{ci}r{ri}", "nA": int(ma.sum()), "nB": int(mb.sum())}
            for K in (10, 25, 60):
                sfx = f"@K{K}"
                for tag, P, C in (("A", Pa, Ra), ("B", Pb, Rb)):
                    for k, v in metrics(P, C, ref, px, K).items():
                        kk = k + sfx if k.startswith(("G", "M10", "O10")) else k
                        rec[f"{tag}_{kk}"] = v
            rec.update(cross_metrics(Pa, Pb, 0))
            rowsout.append(rec)
    np.save("/tmp/qc_rows.npy", np.array(rowsout, dtype=object), allow_pickle=True)
    keys = sorted({k for r in rowsout for k in r if k not in ("patch", "nA", "nB")})
    print(f"patch 数 {len(rowsout)}  指标数 {len(keys)}")
    print("已存 /tmp/qc_rows.npy，用 qc_metric_report.py 出筛选结论")


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    cols = rows = px = None
    for x in sys.argv[1:]:
        if x.startswith("--cols="):
            cols = int(x.split("=")[1])
        if x.startswith("--rows="):
            rows = int(x.split("=")[1])
        if x.startswith("--px="):
            px = float(x.split("=")[1])
    main(a[0], a[1], cols or 8, rows or 2, px or 0.002)
