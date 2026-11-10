#!/bin/bash
python3 - <<'PY'
import sys, csv, gzip
sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
import numpy as np
from voxel_attribution import load_obs, OBS

cls = list(csv.DictReader(open("/home/ywj/slam_data/singleframe/blackframe_replay/attribution/voxel_classification.csv")))
high = {int(r["voxel_id"]) for r in cls if r["group"] == "high"}

vid, fr, P, RGB, EL = load_obs(OBS)
vid_el, fr_el, RGB_el = vid[EL], fr[EL], RGB[EL]
L = 0.299*RGB_el[:,0] + 0.587*RGB_el[:,1] + 0.114*RGB_el[:,2]
mask = np.isin(vid_el, list(high))
v, f, l = vid_el[mask], fr_el[mask], L[mask]
# 体素中位色
order = np.argsort(v, kind="stable")
vs, fs, ls = v[order], f[order], l[order]
uniq, starts, cnt = np.unique(vs, return_index=True, return_counts=True)
med = np.array([np.median(ls[s:s+c]) for s, c in zip(starts, cnt)])
dev = ls - med[np.searchsorted(uniq, vs)]
bad = np.abs(dev) > 40
print(f"high 组观测 {len(v):,}，|dev|>40 占 {bad.mean()*100:.1f}%")
# 按帧归因：每帧贡献的坏观测数 / 该帧在 high 组的观测数
nf_all = np.bincount(f, minlength=40)
nf_bad = np.bincount(f[bad], minlength=40)
rate = np.divide(nf_bad, np.maximum(nf_all, 1))
o = np.argsort(-rate)
print(f"{'帧':>4}{'观测':>8}{'坏观测':>8}{'坏率%':>8}")
for i in o[:12]:
    print(f"{i:>4}{nf_all[i]:>8}{nf_bad[i]:>8}{rate[i]*100:>8.1f}")
print("全体帧坏率中位", f"{np.median(rate[nf_all>0])*100:.1f}%")
# 坏观测的方向性：dev 符号按帧
sg = np.array([np.mean(dev[f == i][np.abs(dev[f == i]) > 40]) if nf_bad[i] else 0 for i in range(40)])
print("各帧坏观测平均偏差(亮/暗) top:", [(i, round(float(sg[i]))) for i in o[:12]])
PY
