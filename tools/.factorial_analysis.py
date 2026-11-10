#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""组合实验分析：椒盐率与 LIO 位姿误差的剂量-响应、相关性与方差分解。
附加：e1 相对 sl 的真实逐帧位姿偏差（把真实 LIO 放到合成剂量轴上）。"""
import json
import sys

import numpy as np
from scipy import stats as sst

sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
from blackframe_voxel_replay import load_bag, make_interpolator

D = json.load(open("/home/ywj/slam_data/singleframe/blackframe_replay/"
                   "lio_color_factorial/summary.json"))
LV = {r["name"]: r for r in D["levels"]}

# ---- e1 真实位姿偏差（vs sl，同数据集时钟） ----
f_sl, o_sl = load_bag("/home/ywj/slam_data/singleframe/sl_frames")
f_e, o_e = load_bag("/home/ywj/slam_data/e1_append/e1_frames")
pose_sl = make_interpolator(o_sl)
idx = np.linspace(0, len(f_e) - 1, 40).astype(int)
st = np.array([o[0] for o in o_e])
sp = np.array([o[1] for o in o_e])
dev = []
for i in idx:
    t = f_e[i][0]
    if t < st[0] or t > st[-1]:
        continue
    if t < o_sl[0][0] or t > o_sl[-1][0]:      # 仅在 sl odom 时钟范围内比较
        continue
    _, tv1 = pose_sl(t)
    j = min(max(int(np.searchsorted(st, t)), 1), len(st) - 1)
    a = (t - st[j - 1]) / (st[j] - st[j - 1])
    pe = sp[j - 1] + a * (sp[j] - sp[j - 1])
    dev.append(np.linalg.norm(pe - tv1))
dev = np.array(dev)
print(f"e1 vs sl 逐帧位置差: 均值 {dev.mean()*100:.1f}cm 中位 {np.median(dev)*100:.1f}cm "
      f"p90 {np.percentile(dev,90)*100:.1f}cm")

# ---- 表 ----
rows = []
for name, r in LV.items():
    p = r["pose"]
    rows.append(dict(
        level=name, obs=r["obs"],
        dev_mean=p["pose_dev_mean_cm"], rel=p["rel_trans_mean_cm"],
        pw=p.get("pairwise_trans_cm", float("nan")),
        pwr=p.get("pairwise_rot_deg", float("nan")),
        collide=r["geo"]["collide_frac"] * 100,
        F=r["cells"]["F"]["rate"], A=r["cells"]["A"]["rate"], G=r["cells"]["G"]["rate"]))

print(f"\n{'level':<12}{'dev':>7}{'rel':>7}{'pairw':>7}{'pwr':>6}"
      f"{'coll%':>7}{'F%':>7}{'A%':>7}{'G%':>7}{'A-F':>6}{'G-F':>6}")
for x in rows:
    print(f"{x['level']:<12}{x['dev_mean']:>7.1f}{x['rel']:>7.1f}{x['pw']:>7.1f}"
          f"{x['pwr']:>6.2f}{x['collide']:>7.1f}{x['F']:>7.2f}{x['A']:>7.2f}"
          f"{x['G']:>7.2f}{x['A']-x['F']:>6.2f}{x['G']-x['F']:>6.2f}")

# ---- 相关性（仅 7 个合成级：观测/颜色完全相同，唯一变量=位姿） ----
allr = [x for x in rows if x["level"] != "e1_real"]
e1r = next(x for x in rows if x["level"] == "e1_real")

for key in ["F", "A", "G"]:
    y = np.array([x[key] for x in allr])
    for xk in ["dev_mean", "rel", "pw"]:
        xx = np.array([x[xk] for x in allr])
        m = np.isfinite(xx)
        pr = sst.pearsonr(xx[m], y[m])
        sr = sst.spearmanr(xx[m], y[m])
        print(f"{key} vs {xk:<9} pearson={pr[0]:+.3f} (p={pr[1]:.1e}) "
              f"spearman={sr[0]:+.3f}")

# 拟合曲线：F 率 ~ 线性(pairwise) → 给 e1 定位
xx = np.array([x["pw"] for x in allr]); yy = np.array([x["F"] for x in allr])
sl_, ic_ = np.polyfit(xx, yy, 1)
pred_e1 = sl_ * dev.mean() * 100 + ic_
print(f"\n拟合 F% = {ic_:.2f} + {sl_:.3f}×pairwise_cm "
      f"(R={sst.pearsonr(xx, yy)[0]:.3f})")
print(f"e1 实测偏差均值 {dev.mean()*100:.1f}cm → 预测 F={pred_e1:.1f}%，"
      f"实测 F={e1r['F']:.1f}%（含密度/帧集差异混杂，仅作参考）")

# ---- 方差分解（双因素：L 级 × C 模型，含 e1，不含缺失） ----
tab = np.array([[x["F"], x["A"], x["G"]] for x in rows])
grand = tab.mean()
ss_L = (len(rows) ** 0) * sum((r.mean() - grand) ** 2 for r in tab) * tab.shape[1]
ss_C = sum((c.mean() - grand) ** 2 for c in tab.T) * tab.shape[0]
ss_tot = ((tab - grand) ** 2).sum()
ss_int = ss_tot - ss_L - ss_C
print(f"\n方差分解 (总范围 {tab.min():.2f}%~{tab.max():.2f}%):")
print(f"  LIO 级间 SS: {ss_tot and ss_L/ss_tot*100:6.2f}%")
print(f"  着色规则间 SS: {ss_C/ss_tot*100:6.2f}%")
print(f"  交互/残差:     {ss_int/ss_tot*100:6.2f}%")
dC = tab.max(1) - tab.min(1)
print(f"  每级内模型极差: {dC.min():.2f}~{dC.max():.2f}pp")
dL = tab.max(0) - tab.min(0)
print(f"  每模型跨级极差: F {dL[0]:.2f}pp, A {dL[1]:.2f}pp, G {dL[2]:.2f}pp")

json.dump(dict(rows=rows, e1_vs_sl_dev_cm=dict(
    mean=float(dev.mean() * 100), median=float(np.median(dev) * 100),
    p90=float(np.percentile(dev, 90) * 100)),
    ss_split=dict(L=ss_L / ss_tot, C=ss_C / ss_tot, resid=ss_int / ss_tot)),
    open("/home/ywj/slam_data/singleframe/blackframe_replay/"
         "lio_color_factorial/analysis.json", "w"), ensure_ascii=False, indent=1)
