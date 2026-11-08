#!/usr/bin/env python3
"""从 /tmp/qc_rows.npy 出筛选结论：区分度、成对一致性、参数鲁棒、冗余聚类。"""
import sys
import numpy as np
from scipy.stats import spearmanr

rows = np.load("/tmp/qc_rows.npy", allow_pickle=True).tolist()
keys = sorted({k[2:] for r in rows for k in r if k.startswith(("A_", "B_"))})
M = []
for k in keys:
    a = np.array([r.get("A_" + k, np.nan) for r in rows], float)
    b = np.array([r.get("B_" + k, np.nan) for r in rows], float)
    ok = np.isfinite(a) & np.isfinite(b)
    if ok.sum() < 6:
        continue
    d = b[ok] - a[ok]
    scale = max(abs(np.mean(a[ok])), abs(np.mean(b[ok])), 1e-12)
    dz = np.mean(d) / (np.std(d) + 1e-12)
    M.append(dict(name=k, n=int(ok.sum()), A=np.mean(a[ok]), B=np.mean(b[ok]),
                  rel=np.mean(d) / scale, cons=np.mean(np.sign(d) == np.sign(np.mean(d))),
                  dz=dz))
M.sort(key=lambda x: -abs(x["dz"]))

print(f"patch 配对 {len(rows)} 组，可用指标 {len(M)} 个\n")
print(f"{'指标':<40}{'A(官方)':>12}{'B(SL)':>12}{'相对差%':>10}{'成对d_z':>10}{'方向一致':>10}")
for m in M:
    print(f"{m['name']:<40}{m['A']:>12.5f}{m['B']:>12.5f}{m['rel']*100:>10.1f}"
          f"{m['dz']:>10.2f}{m['cons']*100:>9.0f}%")

# 参数鲁棒：G 族三个邻居数下的 d_z 符号是否一致
print("\n-- 尺度鲁棒性（同一指标在 K=10/25/60 下的 d_z）--")
base = sorted({k.split("@")[0] for k in keys if "@" in k})
for b in base:
    vs = [next((m["dz"] for m in M if m["name"] == f"{b}@K{r}"), np.nan) for r in (10, 25, 60)]
    flip = len({np.sign(v) for v in vs if np.isfinite(v)}) > 1
    print(f"   {b:<32} {vs[0]:>7.2f} {vs[1]:>7.2f} {vs[2]:>7.2f}  {'排序翻转' if flip else '稳定'}")

# 冗余聚类
print("\n-- Spearman 冗余聚类（|rho|>0.95 归为一族，只留 d_z 最大的代表）--")
names = [m["name"] for m in M]


def series(k):
    v = []
    for r in rows:
        for p in ("A_", "B_"):
            v.append(r.get(p + k, np.nan))
    return np.array(v, float)


mat = np.full((len(names), len(names)), np.nan)
S = [series(k) for k in names]
for i in range(len(names)):
    for j in range(i, len(names)):
        ok = np.isfinite(S[i]) & np.isfinite(S[j])
        if ok.sum() >= 12 and np.std(S[i][ok]) > 0 and np.std(S[j][ok]) > 0:
            rho = spearmanr(S[i][ok], S[j][ok]).statistic
            mat[i, j] = mat[j, i] = rho
groups = []
used = set()
for i, m in enumerate(M):
    if m["name"] in used:
        continue
    g = [m["name"]]
    for j in range(i + 1, len(M)):
        if M[j]["name"] in used or not np.isfinite(mat[i, j]):
            continue
        if abs(mat[i, j]) > 0.95:
            g.append(M[j]["name"])
            used.add(M[j]["name"])
    used.add(m["name"])
    groups.append((m["name"], g))
for rep, g in sorted(groups, key=lambda x: -abs(next(m["dz"] for m in M if m["name"] == x[0]))):
    dz = next(m["dz"] for m in M if m["name"] == rep)
    print(f"   {rep:<36} d_z={dz:>7.2f}  代表 {len(g)} 个: "
          f"{', '.join(x for x in g if x != rep) or '-'}")
