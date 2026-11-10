#!/bin/bash
python3 - <<'PY'
import csv
import numpy as np
p = "/home/ywj/slam_data/singleframe/blackframe_replay/attribution/voxel_classification.csv"
rows = list(csv.DictReader(open(p)))
def col(name, g=None):
    v = [r[name] for r in rows if g is None or r["group"] == g]
    return np.array([np.nan if x == "" else float(x) for x in v], float)

for g in ("high", "low", "boundary"):
    sp, gap, cv, fr5 = (col(x, g) for x in ("frame_centroid_spread_m", "strata_gap_m",
                                            "corr_lum_cosview", "nn"))
    ok = col("plane_ok", g)
    print(f"[{g}] n={len(sp)} plane_ok={ok.mean()*100:.0f}% nn>=5占{(fr5>=5).mean()*100:.0f}%")
    for nm, v, ths in (("质心位移cm", sp*100, (1, 2, 3)), ("分层间隙cm", gap*100, (1, 1.5, 2)),
                       ("corr_cosview", cv, None)):
        if ths:
            print(f"   {nm}: p50={np.nanmedian(v):.2f} p90={np.nanpercentile(v,90):.2f} "
                  + " ".join(f">{t}:{np.mean(v>t)*100:.1f}%" for t in ths))
        else:
            m = np.isfinite(v)
            print(f"   {nm}: 中位 {np.nanmedian(v):.3f} |corr|>0.6 占 {np.mean(np.abs(v[m])>0.6)*100:.0f}% (n={m.sum()})")
    cs = col("corr_lum_dsigned", g)
    print(f"   corr_lum_dsigned: 中位 {np.nanmedian(cs):.3f} |.|>0.6 占 {np.mean(np.abs(cs[np.isfinite(cs)])>0.6)*100:.0f}%")
PY
