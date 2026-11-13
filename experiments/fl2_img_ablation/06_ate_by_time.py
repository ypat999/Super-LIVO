#!/usr/bin/env python3
"""把 NTU eee_03 的 ATE 按 10 s 分箱，定位「有/无图像」的差异到底集中在哪一段。

用法：python3 06_ate_by_time.py
"""
import glob
import os
import sys

import numpy as np

sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
sys.path.insert(0, "/home/ywj/git/Super-LIVO/experiments/fl2_img_ablation")
import odom_vs_gt as og  # noqa: E402

OUT_AB = "/home/ywj/slam_data/fl2_img_abl_20261110"
GT = "/home/ywj/slam_data/ntu_viral/gt/eee_03/ground_truth.csv"
BIN = 10.0


def curve(path):
    est = og.load_odom(path, "/none")
    t = est[:, 0]
    p = est[:, 1:4].copy()
    for i in range(len(p)):
        p[i] += og.quat_to_rot(est[i, 4:8]) @ og.T_B_PRISM
    pg, ok = og.interp_gt_at(GT_ARR, t)
    pe, pgg, te = p[ok], pg[ok], t[ok]
    # 全窗 SE(3) 对齐（与 ATE 同口径），再按 10 s 分箱算 RMSE
    R, tr = og.umeyama_se3(pe, pgg)
    err = np.linalg.norm((R @ pe.T).T + tr - pgg, axis=1)
    t0 = te[0]
    nb = int((te[-1] - t0) // BIN) + 1
    return np.array([np.sqrt(np.mean(err[(te - t0 >= k * BIN) &
                                         (te - t0 < (k + 1) * BIN)] ** 2)) * 1e3
                     if ((te - t0 >= k * BIN) & (te - t0 < (k + 1) * BIN)).sum() > 5 else np.nan
                     for k in range(nb)])


GT_ARR = og.load_gt(GT)
print(f"每格 {BIN:.0f} s 的 ATE(mm)，全窗统一对齐（同 ATE 口径）")
for f in sorted(glob.glob(f"{OUT_AB}/ntu/m*_pose.txt")):
    tag = os.path.basename(f).replace("_pose.txt", "")
    c = curve(f)
    print(f"{tag}  " + "".join(f"{v:6.0f}" if v == v else "     -" for v in c))

for b in range(3):
    k1, k0 = f"m1r{b+1}", f"m0r{b+1}"
    c1 = curve(f"{OUT_AB}/ntu/{k1}_pose.txt")
    c0 = curve(f"{OUT_AB}/ntu/{k0}_pose.txt")
    n = min(len(c0), len(c1))
    print(f"\nrep{b+1} 无图像 − 有图像（mm）：" + "".join(f"{c0[i]-c1[i]:+6.0f}" for i in range(n)))
    print(f"rep{b+1} 0-30s 段：无图像 {np.sqrt(np.mean(c0[:3]**2))*1:.0f} / "
          f"有图像 {np.sqrt(np.mean(c1[:3]**2))*1:.0f} mm；"
          f"30s 之后：无图像 {np.sqrt(np.mean(c0[3:]**2)):.0f} / "
          f"有图像 {np.sqrt(np.mean(c1[3:]**2)):.0f} mm")
