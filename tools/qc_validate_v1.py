#!/usr/bin/env python3
"""Q1-Q4 四项核心指标的独立性与鲁棒性验证（对应 gpt_点云指标.md 的追加实验）。

实验1  G00 与 O02 的条件相关性：按 G00 分层后，O02 是否还能区分 A/B
实验2  G15 密度敏感性：把官方随机下采样到 95/90/87/80/70%，G15 会不会自己涨上去
实验3  栅格域深度分层椒盐：SL 的椒盐是"两层表面投影碰撞"还是"同一表面内的颜色污染"
用法:  python3 qc_validate_v1.py <A.pcd> <B.pcd> [--cell=0.20] [--px=0.002]
"""
import sys
import numpy as np
from scipy.spatial import cKDTree
from scipy.stats import spearmanr
from scipy.ndimage import median_filter, distance_transform_edt

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from qc_metric_screen import read_pcd, local_moments, lum, rgb2lab  # noqa: E402

EPS = 1e-12


def frame(PA, PB):
    allp = np.concatenate([PA, PB])
    c0 = allp.mean(0)
    _, _, vt = np.linalg.svd(allp - c0, full_matrices=False)
    n0 = vt[2]
    if n0[2] < 0:
        n0 = -n0
    a1 = np.array([n0[1], -n0[0], 0.0])
    a1 /= np.linalg.norm(a1) + EPS
    a2 = np.cross(n0, a1)
    return c0, n0, a1, a2


def cell_ids(P, c0, a1, a2, cell):
    v = P - c0
    u, w = v @ a1, v @ a2
    iu = np.floor((u - u.min()) / cell).astype(np.int64)
    iw = np.floor((w - w.min()) / cell).astype(np.int64)
    nx = int(iu.max()) + 1
    cid = iw * nx + iu
    return cid, nx, int(iw.max()) + 1


def g00_o02(P, K=10, r=0.03, minb=6):
    """一次 k=60 查询同时得到 G00（第10近邻距离）与 O02（3cm 内邻居<6 的占比）。"""
    tree = cKDTree(P)
    dd, _ = tree.query(P, k=60)
    g00 = float(dd[:, K - 1].mean())
    o02 = float(np.mean((dd < r).sum(1) < minb))
    return g00, o02


def g15(P, K=25):
    d, mean, lam, nrm, radii = local_moments(P, K)
    rg = np.sqrt(lam[:, 2])
    return float(np.std(rg) / max(np.mean(rg), EPS))


def strat_table(g00a, o02a, g00b, o02b):
    """按 G00 分位分层，比较同层内 A/B 的 O02。"""
    allg = np.concatenate([g00a, g00b])
    qs = np.quantile(allg, [0, 1 / 3, 2 / 3, 1.0])
    print(f"   {'G00 分层(m)':<18}{'A n':>6}{'B n':>6}{'A O02%':>10}{'B O02%':>10}{'B/A':>8}")
    for i in range(3):
        lo, hi = qs[i], qs[i + 1]
        ma = (g00a >= lo) & (g00a <= hi if i == 2 else g00a < hi)
        mb = (g00b >= lo) & (g00b <= hi if i == 2 else g00b < hi)
        if ma.sum() < 3 or mb.sum() < 3:
            continue
        a, b = o02a[ma].mean() * 100, o02b[mb].mean() * 100
        print(f"   {f'{lo*100:.1f}-{hi*100:.1f}cm':<18}{ma.sum():>6}{mb.sum():>6}"
              f"{a:>10.3f}{b:>10.3f}{b/max(a,EPS):>8.2f}")
    # 用 A 拟合 O02~G00（对数-对数），看 B 的实际值超出预测多少
    xa, ya = np.log(g00a + EPS), np.log(o02a + 1e-6)
    xb, yb = np.log(g00b + EPS), np.log(o02b + 1e-6)
    if np.std(xa) > EPS:
        b1, b0 = np.polyfit(xa, ya, 1)
        pred = b0 + b1 * xb
        excess = np.exp(np.mean(yb - pred))
        print(f"   A 上拟合 log(O02)~log(G00): 斜率 {b1:.2f}, R^2="
              f"{np.corrcoef(xa, ya)[0,1]**2:.2f}; B 相对该关系的 O02 超额倍数 = {excess:.2f}x")


def raster(P, rgb, dn, c0, n0, a1, a2, px, mask=None):
    v = P - c0
    u, w = v @ a1, v @ a2
    if mask is not None:
        u, w, dn, rgb = u[mask], w[mask], dn[mask], rgb[mask]
    if len(u) < 500:
        return None
    iu = np.floor((u - u.min()) / px).astype(np.int64)
    iw = np.floor((w - w.min()) / px).astype(np.int64)
    W_, H_ = int(iu.max()) + 1, int(iw.max()) + 1
    if W_ * H_ > 40_000_000:
        return None
    pix = iw * W_ + iu
    zb = np.full(W_ * H_, np.inf)
    np.minimum.at(zb, pix, dn)
    filled = np.isfinite(zb).reshape(H_, W_)
    order = np.argsort(-dn)
    flat = np.zeros(H_ * W_)
    flat[pix[order]] = lum(rgb)[order]
    L = flat.reshape(H_, W_)
    _, near = distance_transform_edt(~filled, return_indices=True)
    L = L[near[0], near[1]]
    return L, filled, zb, pix


def salt_pepper(L, filled, thr=40):
    med = median_filter(L, 5, mode="nearest")
    return float(np.mean((np.abs(L - med) > thr)[filled]))


def main(pa, pb, cell=0.20, px=0.002):
    PA, RA = read_pcd(pa)
    PB, RB = read_pcd(pb)
    c0, n0, a1, a2 = frame(PA, PB)
    print(f"A {len(PA):,} 点   B {len(PB):,} 点   cell {cell*100:.0f}cm")

    # ---------------- 实验 1 ----------------
    print("\n=== 实验1：G00 与 O02 的条件相关性 ===")
    ca, _, _ = cell_ids(PA, c0, a1, a2, cell)
    cb, _, _ = cell_ids(PB, c0, a1, a2, cell)
    ga, oa, gb, ob = [], [], [], []
    for cid in np.unique(np.concatenate([ca, cb])):
        ma, mb = ca == cid, cb == cid
        if ma.sum() < 250 or mb.sum() < 250:
            continue
        x, y = g00_o02(PA[ma]);  ga.append(x); oa.append(y)
        x, y = g00_o02(PB[mb]);  gb.append(x); ob.append(y)
    ga, oa, gb, ob = map(np.array, (ga, oa, gb, ob))
    print(f"   有效 cell {len(ga)}  G00 均值 A {ga.mean()*100:.2f}cm / B {gb.mean()*100:.2f}cm"
          f"   O02 均值 A {oa.mean()*100:.3f}% / B {ob.mean()*100:.3f}%")
    print(f"   Spearman(G00,O02): 合并 {spearmanr(np.r_[ga,gb], np.r_[oa,ob]).statistic:.3f}"
          f"  仅A {spearmanr(ga, oa).statistic:.3f}  仅B {spearmanr(gb, ob).statistic:.3f}")
    strat_table(ga, oa, gb, ob)

    # ---------------- 实验 2 ----------------
    print("\n=== 实验2：G15 密度敏感性（官方随机下采样，每档 20 次）===")
    print(f"   {'保留比例':<10}{'G15 均值':>10}{'G15 std':>10}{'G00 cm':>10}{'O02 %':>10}")
    rng = np.random.default_rng(1)
    for ratio in (1.00, 0.95, 0.90, 0.87, 0.80, 0.70):
        vals, gvals, ovals = [], [], []
        reps = 20 if ratio < 1.0 else 1
        for _ in range(reps):
            sel = rng.random(len(PA)) < ratio if ratio < 1.0 else np.ones(len(PA), bool)
            Ps, cs = PA[sel], ca[sel]
            sub_g, sub_o, sub_15 = [], [], []
            for cid in np.unique(cs):
                m = cs == cid
                if m.sum() < 250:
                    continue
                x, y = g00_o02(Ps[m])
                sub_g.append(x); sub_o.append(y); sub_15.append(g15(Ps[m]))
            if len(sub_15) >= 20:
                vals.append(np.mean(sub_15)); gvals.append(np.mean(sub_g)); ovals.append(np.mean(sub_o))
        if vals:
            print(f"   {ratio*100:>5.0f}%    {np.mean(vals):>10.4f}{np.std(vals):>10.4f}"
                  f"{np.mean(gvals)*100:>10.2f}{np.mean(ovals)*100:>10.3f}")
    b15 = np.mean([g15(PB[cb == cid]) for cid in np.unique(cb)
                   if (cb == cid).sum() >= 250])
    b00, bo2 = np.mean([g00_o02(PB[cb == cid])[0] for cid in np.unique(cb) if (cb == cid).sum() >= 250]), None
    print(f"   B(SL) 实测 G15 = {b15:.4f}   G00 = {b00*100:.2f} cm")

    # ---------------- 实验 3 ----------------
    print("\n=== 实验3：栅格域深度分层椒盐 ===")
    for tag, P, R in (("A 官方", PA, RA), ("B SL", PB, RB)):
        v = P - c0
        dn = v @ n0
        res = raster(P, R, dn, c0, n0, a1, a2, px)
        if res is None:
            continue
        L, filled, zb, pix = res
        behind = dn - zb[pix]
        print(f"   {tag}: 全点椒盐 {salt_pepper(L, filled)*100:5.2f}%")
        for lo, hi, nm in ((0.0, 0.01, "皮层 0-1cm"), (0.01, 0.04, "浅后层 1-4cm"),
                           (0.04, 0.08, "深后层 4-8cm"), (0.08, 9.9, "远后层 >8cm"),
                           (-9.9, 0.0, "皮前层(在皮之前)")):
            m = (behind >= lo) & (behind < hi)
            r2 = raster(P, R, dn, c0, n0, a1, a2, px, mask=m)
            if r2 is None:
                continue
            L2, f2, _, _ = r2
            print(f"        {nm:<18} 点 {m.sum():>8,} ({m.mean()*100:5.1f}%)"
                  f"  该层自身椒盐 {salt_pepper(L2, f2)*100:5.2f}%")
        # 只保留"贴在局部表面上"的点（局部平面残差 <1.5cm）
        d, mean, lam, nrm, radii = local_moments(P, 25)
        lp = np.abs(((P - mean) * nrm).sum(1))
        m = lp < 0.015
        r2 = raster(P, R, dn, c0, n0, a1, a2, px, mask=m)
        if r2:
            L2, f2, _, _ = r2
            print(f"        仅局部贴合点(|残差|<1.5cm) {m.mean()*100:5.1f}%  "
                  f"椒盐 {salt_pepper(L2, f2)*100:5.2f}%")


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    cell, px = 0.20, 0.002
    for x in sys.argv[1:]:
        if x.startswith("--cell="):
            cell = float(x.split("=")[1])
        if x.startswith("--px="):
            px = float(x.split("=")[1])
    main(a[0], a[1], cell, px)
