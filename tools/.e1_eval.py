#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E1 评估：纯追加地图 vs 融合回填地图，统一栅格口径。

口径（与 .fusion_effect.sh 的融合地图视图一致，但掩码改为"本帧已上色点覆盖"，
两套地图各用自己的 bag 位姿与掩码规则，规则完全相同）：
  对每个录制帧 t：pose_at(t) → 把 PCD 全部点变到相机系投影 → 像素 z-buffer
  取最近点色 → EDT 补洞 → median_filter(5) |Δ|>40 计椒盐；
  掩码 = 本帧自身上色点（rgb!=0）投影像素 5x5 膨胀。
另输出：逐帧观测帧数栅格（1cm 体素 nframes）、两跑轨迹一致性（odom ATE）、
并排栅格颜色图。
"""
import json
import os
import re
import sys

import numpy as np
from scipy.ndimage import median_filter, convolve, distance_transform_edt

sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
from blackframe_voxel_replay import load_bag, make_interpolator, read_pcd_xyzrgb, lum_of

VOX = 0.01
JUMP = 40.0
ZMIN = 0.3

yaml = open("/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml").read()
fx = float(re.search(r"cam_fx:\s*([\d.]+)", yaml).group(1))
fy = float(re.search(r"cam_fy:\s*([\d.]+)", yaml).group(1))
cx = float(re.search(r"cam_cx:\s*([\d.]+)", yaml).group(1))
cy = float(re.search(r"cam_cy:\s*([\d.]+)", yaml).group(1))
W = int(re.search(r"cam_width:\s*(\d+)", yaml).group(1))
H = int(re.search(r"cam_height:\s*(\d+)", yaml).group(1))
Rcl = np.array([float(v) for v in re.search(r"Rcl:\s*\[([^\]]+)\]", yaml, re.S).group(1).replace("\n", " ").split(",")]).reshape(3, 3)
Pcl = np.array([float(v) for v in re.search(r"Pcl:\s*\[([^\]]+)\]", yaml).group(1).split(",")])


def project(Pw, R, tv):
    p_cam = (Pw - tv) @ R @ Rcl.T + Pcl
    z = p_cam[:, 2]
    m = z > ZMIN
    u = fx * p_cam[m, 0] / z[m] + cx
    v = fy * p_cam[m, 1] / z[m] + cy
    mm = (u >= 1) & (u < W - 1) & (v >= 1) & (v < H - 1)
    return u[mm], v[mm], z[m][mm], m, mm


def eval_map(pcd_path, bag_path, out_dir, n_hist_frames=None, hist_ts=None):
    frames, odom = load_bag(bag_path)
    pose_at = make_interpolator(odom)
    Pmap, rgbmap = read_pcd_xyzrgb(pcd_path)
    Lum = lum_of(rgbmap)
    Pmap = Pmap.astype(np.float64)

    # 观测帧数：本 bag 上色点按 1cm 体素的唯一帧数
    all_u, all_v = [], []
    vox_frames = {}
    for fi, (t, Pb, pk) in enumerate(frames):
        R, tv = pose_at(t)
        Pw = Pb.astype(np.float64) @ R.T + tv
        col = lum_of(pk) > 0
        key = np.floor(Pw[col] / VOX).astype(np.int64)
        kk = (key[:, 0] * 1000003 + key[:, 1]) * 1000003 + key[:, 2]
        for k in np.unique(kk):
            vox_frames.setdefault(int(k), set()).add(fi)
    nvox = {k: len(v) for k, v in vox_frames.items()}

    tot = jump_sum = covsum = 0
    salt_multi = salt_single = 0
    per_frame = []
    imgs = []
    for fi, (t, Pb, pk) in enumerate(frames):
        R, tv = pose_at(t)
        # 掩码：本帧上色点覆盖
        col = lum_of(pk) > 0
        Pc = Pb.astype(np.float64) @ R.T + tv
        uo, vo, zo, _, _ = project(Pc[col], R, tv)
        cov = np.zeros((H, W), bool)
        cov[np.floor(vo).astype(np.int64), np.floor(uo).astype(np.int64)] = True
        cc = convolve(cov.astype(np.float32), np.ones((5, 5)), mode="constant") >= 1

        u, v, zz, mpre, mpost = project(Pmap, R, tv)
        # 与 pre 过滤对齐的索引
        idx_all = np.nonzero(mpre)[0]
        idx_all = idx_all[mpost]
        im = np.floor(v).astype(np.int64) * W + np.floor(u).astype(np.int64)
        zb = np.full(W * H, np.inf)
        np.minimum.at(zb, im, zz)
        filled = np.isfinite(zb)
        win = np.full(W * H, -1, np.int64)
        ordz = np.argsort(-zz)
        win[im[ordz]] = idx_all[ordz]
        Lpix = np.zeros(W * H)
        Npix = np.zeros(W * H)
        Lpix[filled] = Lum[win[filled]]
        if filled.any():
            wk = np.floor(Pmap[win[filled]] / VOX).astype(np.int64)
            kk = (wk[:, 0] * 1000003 + wk[:, 1]) * 1000003 + wk[:, 2]
            Npix[filled] = [nvox.get(int(k), 1) for k in kk]
        Limg = Lpix.reshape(H, W)
        Nimg = Npix.reshape(H, W)
        _, near = distance_transform_edt(~filled.reshape(H, W), return_indices=True)
        Lf = Limg[near[0], near[1]]
        Nf = Nimg[near[0], near[1]]
        jump = (np.abs(Lf - median_filter(Lf, 5, mode="nearest")) > JUMP) & cc
        tot += int(jump.sum())
        covsum += int(cc.sum())
        salt_multi += int((jump & (Nf > 1)).sum())
        salt_single += int((jump & (Nf == 1)).sum())
        per_frame.append((int(jump.sum()), int(cc.sum())))
        pick = (n_hist_frames and fi in n_hist_frames) or \
               (hist_ts and min(abs(t - x) for x in hist_ts) < 0.06)
        if pick:
            imgs.append((fi, t, Limg, Nimg, cc, jump))

    rate = tot / max(covsum, 1) * 100
    return dict(pcd=pcd_path, salt=tot, cover=covsum, rate=rate,
                salt_multi_obs=salt_multi, salt_single_obs=salt_single,
                multi_share=salt_multi / max(tot, 1) * 100,
                per_frame=per_frame), imgs, nvox


def odom_ate(bag_a, bag_b):
    _, oa = load_bag(bag_a)
    _, ob = load_bag(bag_b)
    ta = np.array([o[0] for o in oa]); pa = np.array([o[1] for o in oa])
    tb = np.array([o[0] for o in ob]); pb = np.array([o[1] for o in ob])
    # 各自减首帧（两跑世界系原点不同）
    pa = pa - pa[0]; pb = pb - pb[0]
    t0 = max(ta[0], tb[0]); t1 = min(ta[-1], tb[-1])
    ts = np.linspace(t0, t1, 500)
    ia = np.stack([np.interp(ts, ta, pa[:, k]) for k in range(3)], 1)
    ib = np.stack([np.interp(ts, tb, pb[:, k]) for k in range(3)], 1)
    d = np.linalg.norm(ia - ib, axis=1)
    return float(np.sqrt((d ** 2).mean())), float(np.median(d)), float(d.max())


if __name__ == "__main__":
    SL = "/home/ywj/slam_data/singleframe"
    E1 = "/home/ywj/slam_data/e1_append"
    out = os.path.join(E1, "e1_eval")
    os.makedirs(out, exist_ok=True)
    frames_sel = {0, 10, 20, 30}  # sl bag 40 帧中的采样；E1 帧按时间戳就近配对

    print("== 评估 线上融合地图 (sl_map, median 回填) ==")
    res_sl, imgs_sl, nvox_sl = eval_map(f"{SL}/sl_map/super_livo_visual_a_colored.pcd",
                                        f"{SL}/sl_frames", out, frames_sel)
    print(f"  椒盐 {res_sl['salt']:,}/{res_sl['cover']:,} = {res_sl['rate']:.2f}%  "
          f"多观测占比 {res_sl['multi_share']:.1f}%")

    print("== 评估 E1 纯追加地图 (median 回填关闭) ==")
    e1_ts = [x[1] for x in imgs_sl]
    res_e1, imgs_e1, nvox_e1 = eval_map(f"{E1}/e1_map/super_livo_visual_a_colored.pcd",
                                        f"{E1}/e1_frames", out, hist_ts=e1_ts)
    print(f"  椒盐 {res_e1['salt']:,}/{res_e1['cover']:,} = {res_e1['rate']:.2f}%  "
          f"多观测占比 {res_e1['multi_share']:.1f}%")

    ate_rms, ate_med, ate_max = odom_ate(f"{SL}/sl_frames", f"{E1}/e1_frames")
    print(f"== 两跑轨迹一致性: ATE rms={ate_rms*100:.2f}cm med={ate_med*100:.2f}cm max={ate_max*100:.2f}cm ==")

    nv = np.array(list(nvox_e1.values()))
    obs_hist = {f"n={k}": int((nv == k).sum()) for k in (1, 2, 3, 4)}
    obs_hist["n>=5"] = int((nv >= 5).sum())
    print("观测帧数分布(E1 自身):", obs_hist)

    # 并排栅格图：融合色 | E1色 | E1观测帧数热图（椒盐像素红框叠加）
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for (fi, ts_, Ls, Ns, ccs, js), (fei, te, Le, Ne, cce, je) in zip(imgs_sl, imgs_e1):
        fig, ax = plt.subplots(3, 2, figsize=(16, 12))
        for col, (L, N, cc, j, tag) in enumerate([(Ls, Ns, ccs, js, "median-fused (sl_map)"),
                                                  (Le, Ne, cce, je, "pure-append (E1)")]):
            ax[0, col].imshow(L, cmap="gray", vmin=0, vmax=255)
            ax[0, col].set_title(f"frame#{fi} raster color  {tag}")
            rgbim = np.dstack([L, L, L]).astype(np.uint8)
            rgbim[j] = [255, 0, 0]
            ax[1, col].imshow(rgbim)
            ax[1, col].set_title(f"salt pixels (red), n={int(j.sum())}")
            ax[2, col].imshow(np.ma.masked_where(N <= 0, N), cmap="turbo")
            ax[2, col].set_title("per-pixel winner voxel obs-frame-count")
            for a in ax[:, col]:
                a.axis("off")
        fig.tight_layout()
        fig.savefig(os.path.join(out, f"raster_frame{fi:02d}.png"), dpi=110)
        plt.close(fig)

    summary = dict(
        metric="fused-map-view raster: all-PCD z-buffer + EDT fill + median5 |dL|>40, "
               "mask = own-frame colored points 5x5 dilated",
        sl_fused=res_sl, e1_append=res_e1,
        trajectory_ate=dict(rms_m=ate_rms, median_m=ate_med, max_m=ate_max),
        obs_frame_hist_e1=obs_hist,
        reference=dict(offline_replay_A_med6=4.96, offline_replay_F_per_frame=1.17),
    )
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(summary, f, ensure_ascii=False, indent=1, default=str)
    print("== 判定 ==")
    print(f"  E1 纯追加 {res_e1['rate']:.2f}% vs 线上融合 {res_sl['rate']:.2f}% "
          f"(离线参照: F=1.17% A=4.96%)")
