#!/usr/bin/env python3
"""实验4 的替代验证：没有 5-10 组独立扫描，用 patch 级 bootstrap + 对半分割
给出四个核心指标的置信区间与方向一致性，并明确其局限。

Q4（栅格椒盐）在 qc_rows.npy 里没有逐 patch 值，这里按同样的 8x2 网格补算。

用法: python3 qc_bootstrap.py <A.pcd> <B.pcd>
"""
import sys
import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from qc_metric_screen import read_pcd  # noqa: E402
from qc_validate_v1 import frame, cell_ids, raster, salt_pepper  # noqa: E402

CORE = ["G00_Nb_Radius_mm@K10", "G15_Roughness_CV@K25", "O02_ROR_Ratio_3cm",
        "Q4_Raster_SaltPepper", "D05_NN_P95"]
BETTER_UP = {"Q4_Raster_SaltPepper": False, "D05_NN_P95": False,
             "G00_Nb_Radius_mm@K10": False, "G15_Roughness_CV@K25": False,
             "O02_ROR_Ratio_3cm": False}


def per_patch_raster(pa, pb, cols=8, rows=2, px=0.002):
    PA, RA = read_pcd(pa)
    PB, RB = read_pcd(pb)
    c0, n0, a1, a2 = frame(PA, PB)
    vA, vB = PA - c0, PB - c0
    uA, wA = vA @ a1, vA @ a2
    uB, wB = vB @ a1, vB @ a2
    lo, hi = min(uA.min(), uB.min()), max(uA.max(), uB.max())
    wlo, whi = min(wA.min(), wB.min()), max(wA.max(), wB.max())
    out = []
    for ci in range(cols):
        for ri in range(rows):
            u0, u1 = lo + (hi - lo) * ci / cols, lo + (hi - lo) * (ci + 1) / cols
            w0, w1 = wlo + (whi - wlo) * ri / rows, wlo + (whi - wlo) * (ri + 1) / rows
            ma = (uA >= u0) & (uA < u1) & (wA >= w0) & (wA < w1)
            mb = (uB >= u0) & (uB < u1) & (wB >= w0) & (wB < w1)
            if ma.sum() < 3000 or mb.sum() < 3000:
                continue
            ra = raster(PA[ma], RA[ma], (vA[ma]) @ n0, c0, n0, a1, a2, px)
            rb = raster(PB[mb], RB[mb], (vB[mb]) @ n0, c0, n0, a1, a2, px)
            if ra is None or rb is None:
                continue
            out.append((f"c{ci}r{ri}", ci,
                        salt_pepper(*ra[:2]), salt_pepper(*rb[:2])))
    return out


def dz(a, b, idx):
    d = b[idx] - a[idx]
    return np.mean(d) / (np.std(d) + 1e-12)


def main(pa, pb):
    rows = np.load("/tmp/qc_rows.npy", allow_pickle=True).tolist()
    names = [r["patch"] for r in rows]
    data = {}
    for k in CORE:
        a = np.array([r.get("A_" + k, np.nan) for r in rows], float)
        b = np.array([r.get("B_" + k, np.nan) for r in rows], float)
        if np.isfinite(a).all() and np.isfinite(b).all():
            data[k] = (a, b)
    sp = per_patch_raster(pa, pb)
    if len(sp) >= 6:
        names = [x[0] for x in sp]
        data["Q4_Raster_SaltPepper"] = (np.array([x[2] for x in sp]),
                                        np.array([x[3] for x in sp]))
    cols = np.array([int(n[1]) for n in names])
    print(f"patch 数 {len(names)}（列 {sorted(set(cols))}）")

    rng = np.random.default_rng(0)
    print(f"\n{'指标':<26}{'A':>10}{'B':>10}{'d_z':>8}{'95% CI':>18}{'P(d_z>0)':>10}"
          f"{'B更差patch':>12}{'奇半':>8}{'偶半':>8}")
    for k in CORE:
        if k not in data:
            print(f"{k:<26}  无逐 patch 数据")
            continue
        a, b = data[k]
        n = len(a)
        idx = np.arange(n)
        point = dz(a, b, idx)
        bs = np.array([dz(a, b, rng.choice(idx, n, replace=True)) for _ in range(2000)])
        lo, hi = np.percentile(bs, [2.5, 97.5])
        worse = np.mean(b > a) if BETTER_UP.get(k, True) else np.mean(b < a)
        odd, even = idx[cols % 2 == 1], idx[cols % 2 == 0]
        do = dz(a, b, odd) if len(odd) >= 3 else np.nan
        de = dz(a, b, even) if len(even) >= 3 else np.nan
        print(f"{k:<26}{a.mean():>10.4f}{b.mean():>10.4f}{point:>8.2f}"
              f"{f'[{lo:.2f},{hi:.2f}]':>18}{np.mean(bs > 0)*100:>9.0f}%"
              f"{worse*100:>11.0f}%{do:>8.2f}{de:>8.2f}")
    print("\n局限：以上只是同一场景内的空间重采样，不能替代跨扫描复现；"
          "它检验的是\"该差异在本 ROI 内是否稳健\"，不是\"换个场景是否还成立\"。")


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    main(a[0], a[1])
