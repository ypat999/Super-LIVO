#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线 G 实验：仅在几何与颜色一致时才融合（纯离线模拟，不改任何生产代码）。

复用同一观测集（voxel_observations.csv.gz 中 pool_eligible=1 的样本），
比较三种体素颜色模型，评价口径 = .fusion_effect.sh 的融合地图视图
（全部入池观测从每个相机位姿投影，z-buffer 取胜者，当前帧覆盖 5x5 膨胀
掩码内 |L - median5| > 40 记椒盐）：

  F  逐帧独立颜色，不回填          —— 基准（获胜像素取其观测自身颜色）
  A  现行算法：K=6 环形缓冲逐通道中位数
  G  一致性门控融合：按插入序，首观测定基线；后续观测需同时满足
       几何一致 |d_signed - 已接受均值| <= D_THR（局部平面来自
       5x5x5 邻域体素质心加权拟合，同 voxel_attribution 思路）
       颜色一致 max通道|rgb - 已接受中位数| <= C_THR
     才并入融合集；最终色 = 已接受观测逐通道中位数。
     无法拟合平面（邻域点<12）时几何门自动放行（只受颜色门约束）。

阈值敏感性 2x2：D_THR ∈ {0.005, 0.010} m × C_THR ∈ {25, 40}。

除三行椒盐率外，按用户要求统计：
  1) 实际融合的观测比例（后续观测被接受并入的比例）
  2) 被拒绝的冲突观测比例（按拒绝原因分解）
  3) 融合后颜色误差：幽灵深度 min_i|lum_final - lum_i|（最终色离
     任何一次真实观测有多远）+ |lum_final - lum_末次观测|（偏离当前外观）
  4) 颜色一致但几何不一致的案例数
  5) 几何一致但颜色不一致的案例数
（4/5 即几何×颜色一致性 2x2 矩阵。）

输出：fusion_G/summary.json + 控制台。
"""
import json
import os
import sys
import time
import re

import numpy as np
from scipy.ndimage import median_filter, convolve, distance_transform_edt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from voxel_attribution import load_obs, voxel_table, pack16, unpack_vox, OBS, VOX
from blackframe_voxel_replay import (load_bag, make_interpolator, read_pcd_xyzrgb,
                                     QMEM, group_stat, lum_rgb3)

BAG = "/home/ywj/slam_data/singleframe/sl_frames"
MAP = "/home/ywj/slam_data/singleframe/sl_map/super_livo_visual_a_colored.pcd"
PARAMS = "/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml"
OUT = "/home/ywj/slam_data/singleframe/blackframe_replay/fusion_G"

K_RING = 6
JUMP_THR = 40.0
GEOS = [0.005, 0.010]     # m
COLS = [25, 40]           # 0-255
CFG_NAMES = [f"G_d{int(d*1000)}mm_c{c}" for d in GEOS for c in COLS]
DEFAULT_CFG = 0           # (5mm, 40)


# ------------------------------------------------------------------ 局部平面
def fit_planes(uniq, cent, n, multi):
    """5x5x5 邻域体素质心（权重=观测数）加权拟合平面。返回 (c, nrm, plane_ok)。"""
    vox_u = unpack_vox(uniq[multi])
    offs = [(a, b, c) for a in range(-2, 3) for b in range(-2, 3) for c in range(-2, 3)]
    M = len(multi)
    Q = np.full((M, len(offs), 3), np.nan)
    Wt = np.zeros((M, len(offs)))
    for oi, o in enumerate(offs):
        nk = pack16(vox_u + np.array(o))
        j = np.searchsorted(uniq, nk)
        jc = np.clip(j, 0, len(uniq) - 1)
        hit = uniq[jc] == nk
        Q[hit, oi] = cent[jc[hit]]
        Wt[hit, oi] = n[jc[hit]]
    w = np.nan_to_num(Wt)
    Qz = np.nan_to_num(Q)
    sw = w.sum(1)
    c0 = (Qz * w[:, :, None]).sum(1) / np.maximum(sw, 1e-9)[:, None]
    Qc = Qz - c0[:, None, :]
    cov = np.einsum("mpk,mpj,mp->mkj", Qc, Qc, w) / np.maximum(sw, 1e-9)[:, None, None]
    evals, evecs = np.linalg.eigh(cov)
    nrm = evecs[:, :, 0]                      # 最小特征向量的方向 = 法向
    plane_ok = (np.isfinite(sw) & (sw >= 12)) & (np.abs(nrm).sum(1) > 0.5)
    return c0, nrm, plane_ok


# ------------------------------------------------------------------ G 顺序融合
def run_G(n, rgb_s, starts, d_signed, plane_ok_multi, multi, D_THR, C_THR):
    """返回 (final_rgb (m,3), stats dict)。插入序=帧序+帧内序（文件行序）。"""
    m = len(n)
    final = np.zeros((m, 3))
    single = np.where(n == 1)[0]
    final[single] = rgb_s[starts[single]]
    # 2x2 矩阵计数（对每个多观测体素的"后续观测"）
    mat = np.zeros((2, 2), np.int64)   # [geo_ok][col_ok]
    acc_cnt = rej_cnt = sub_cnt = 0
    for gi, g in enumerate(multi):
        r0, nn = starts[g], n[g]
        rgbs = rgb_s[r0:r0 + nn]
        ds = d_signed[gi]
        ok = bool(plane_ok_multi[gi])
        acc = [0]
        med = rgbs[0].copy()
        ref_d = ds[0] if ok else 0.0
        for i in range(1, nn):
            geo_ok = (not ok) or bool(abs(ds[i] - ref_d) <= D_THR)
            col_ok = bool(np.abs(rgbs[i] - med).max() <= C_THR)
            mat[int(geo_ok)][int(col_ok)] += 1
            sub_cnt += 1
            if geo_ok and col_ok:
                acc.append(i)
                med = np.median(rgbs[acc], 0)
                ref_d = float(np.mean(ds[acc])) if ok else 0.0
        final[g] = med
        acc_cnt += len(acc) - 1          # 只计后续观测
        rej_cnt += nn - len(acc)
    stats = dict(matrix=mat.tolist(), subsequent=int(sub_cnt),
                 accepted=int(acc_cnt), rejected=int(rej_cnt))
    return final, stats


def ghost_and_stale(n, starts, rgb_s, final_rgb, multi):
    """多观测体素：幽灵深度 min_i|lum_final-lum_i|；偏离末次 |lum_final-lum_last|。"""
    gh, st = [], []
    for g in multi:
        r0, nn = starts[g], n[g]
        Lo = lum_rgb3(rgb_s[r0:r0 + nn])
        Lf = lum_rgb3(final_rgb[g:g + 1])[0]
        gh.append(np.abs(Lf - Lo).min())
        st.append(abs(Lf - Lo[-1]))
    return np.array(gh), np.array(st)


# ------------------------------------------------------------------ 栅格评价
def raster_eval(P_el, RGB_el, fr_el, lumA, lumG, n, uniq, odom_frames, pose_at,
                cam):
    fx, fy, cx, cy, W, H, Rcl, Pcl = cam
    allvox = np.floor(P_el / VOX).astype(np.int32)
    allg = np.searchsorted(uniq, pack16(allvox))
    L_el = lum_rgb3(RGB_el)
    # 当前帧点来自地图（同 .fusion_effect.sh：用 sl_map 做掩码来源）
    Pmap, rgbmap = read_pcd_xyzrgb(MAP)
    Pmap = Pmap.astype(np.float64)
    def packq(q, bits):
        a = q.astype(np.int64) + (1 << (bits - 1))
        return (a[:, 0] << (2 * bits)) | (a[:, 1] << bits) | a[:, 2]
    uniq_map, idx_map = np.unique(packq(np.round(Pmap * QMEM).astype(np.int32), 21),
                                  return_index=True)
    variants = ["F", "A", "G"] + [f"G:{nm}" for nm in CFG_NAMES]
    tot = {v: 0 for v in variants}
    tot_multi = {v: 0 for v in variants}
    tot_single = {v: 0 for v in variants}
    covsum = 0
    imb_all = n[allg] >= 2
    for fi, (t, Pb, pk) in enumerate(odom_frames):
        R, tv = pose_at(t)
        Pw = Pb.astype(np.float64) @ R.T + tv
        keys = packq(np.round(Pw * QMEM).astype(np.int32), 21)
        j = np.searchsorted(uniq_map, keys)
        el = uniq_map[np.clip(j, 0, len(uniq_map) - 1)] == keys
        Po = Pw[el]
        p_own = (Po - tv) @ R @ Rcl.T + Pcl
        zo = p_own[:, 2]; mo = zo > 0.3
        uo = fx * p_own[mo, 0] / zo[mo] + cx
        vo = fy * p_own[mo, 1] / zo[mo] + cy
        okc = (uo >= 2) & (uo < W - 2) & (vo >= 2) & (vo < H - 2)
        cov = np.zeros((H, W), bool)
        cov[np.floor(vo[okc]).astype(np.int64), np.floor(uo[okc]).astype(np.int64)] = True
        cc = convolve(cov.astype(np.float32), np.ones((5, 5)), mode="constant") >= 1
        p_cam = (P_el - tv) @ R @ Rcl.T + Pcl
        z = p_cam[:, 2]; m = z > 0.3
        u = fx * p_cam[m, 0] / z[m] + cx
        v = fy * p_cam[m, 1] / z[m] + cy
        mm = (u >= 1) & (u < W - 1) & (v >= 1) & (v < H - 1)
        u, v, zz = u[mm], v[mm], z[m][mm]
        gidx = allg[m][mm]
        Lproj = L_el[m][mm]
        im = np.floor(v).astype(np.int64) * W + np.floor(u).astype(np.int64)
        zb = np.full(W * H, np.inf)
        np.minimum.at(zb, im, zz)
        filled = np.isfinite(zb)
        win = np.full(W * H, -1, np.int64)
        ordz = np.argsort(-zz)
        win[im[ordz]] = np.arange(len(im))[ordz]
        wi = win[filled]
        gwin = gidx[wi]
        Mfull = np.zeros(W * H, bool)
        Mfull[filled] = imb_all[gwin]
        _, near = distance_transform_edt(~filled.reshape(H, W), return_indices=True)
        Mf = Mfull.reshape(H, W)[near[0], near[1]]
        covsum += int(cc.sum())
        color_src = {"F": Lproj[wi], "A": lumA[gwin], "G": lumG[DEFAULT_CFG][gwin]}
        for ci, nm in enumerate(CFG_NAMES):
            color_src[f"G:{nm}"] = lumG[ci][gwin]
        for kname, Lw in color_src.items():
            Lpix = np.zeros(W * H)
            Lpix[filled] = Lw
            Limg = Lpix.reshape(H, W)
            Lf = Limg[near[0], near[1]]
            jump = (np.abs(Lf - median_filter(Lf, 5, mode="nearest")) > JUMP_THR) & cc
            tot[kname] += int(jump.sum())
            tot_multi[kname] += int((jump & Mf).sum())
            tot_single[kname] += int((jump & ~Mf).sum())
    return dict(variants=variants, tot=tot, tot_multi=tot_multi,
                tot_single=tot_single, covsum=covsum)


# ------------------------------------------------------------------ 主流程
def main():
    t0 = time.time()
    os.makedirs(OUT, exist_ok=True)
    vid, fr, P, RGB, EL = load_obs(OBS)
    vid_el, fr_el, P_el, RGB_el = vid[EL], fr[EL], P[EL], RGB[EL]
    print(f"观测 {len(vid):,} 行，入池 {len(vid_el):,}")
    tb = voxel_table(vid_el, fr_el, P_el, RGB_el)
    uniq, inv, n, cent = tb["uniq"], tb["inv"], tb["n"], tb["cent"]
    multi = np.where(n >= 2)[0]
    print(f"体素 {len(uniq):,}，多观测 {len(multi):,}，多观测内观测 "
          f"{int(n[multi].sum()):,}（后续 {int(n[multi].sum() - len(multi)):,}）")

    order = np.argsort(inv, kind="stable")
    starts = np.zeros(len(uniq), np.int64)
    np.cumsum(n[:-1], out=starts[1:])
    rgb_s = RGB_el[order]

    # A：K=6 环形中位数（现行算法）
    med6 = group_stat(inv, len(uniq), n, RGB_el, K_RING)
    lumA = lum_rgb3(med6)

    # 几何量：局部平面 + 逐观测有符号距离
    c0, nrm, plane_ok = fit_planes(uniq, cent, n, multi)
    print(f"平面拟合成功 {int(plane_ok.sum()):,}/{len(multi):,} "
          f"({plane_ok.mean()*100:.1f}%)")
    d_signed = []
    for gi, g in enumerate(multi):
        r0, nn = starts[g], n[g]
        if plane_ok[gi]:
            d_signed.append((P_el[order[r0:r0 + nn]] - c0[gi]) @ nrm[gi])
        else:
            d_signed.append(np.zeros(nn))
    d_signed = np.array(d_signed, object)

    results = {}
    lumG = []
    for di, D_THR in enumerate(GEOS):
        for ci, C_THR in enumerate(COLS):
            nm = CFG_NAMES[di * len(COLS) + ci]
            final, st = run_G(n, rgb_s, starts, d_signed, plane_ok, multi, D_THR, C_THR)
            gh, stale = ghost_and_stale(n, starts, rgb_s, final, multi)
            ghA, staleA = ghost_and_stale(n, starts, rgb_s, med6, multi)
            results[nm] = dict(
                D_THR=D_THR, C_THR=C_THR,
                matrix=st["matrix"], subsequent=st["subsequent"],
                accepted=st["accepted"], rejected=st["rejected"],
                accept_rate=st["accepted"] / max(st["subsequent"], 1),
                reject_rate=st["rejected"] / max(st["subsequent"], 1),
                ghost_G=dict(mean=float(gh.mean()), median=float(np.median(gh)),
                             p90=float(np.percentile(gh, 90))),
                ghost_A=dict(mean=float(ghA.mean()), median=float(np.median(ghA)),
                             p90=float(np.percentile(ghA, 90))),
                stale_G=dict(mean=float(stale.mean()), median=float(np.median(stale))),
                stale_A=dict(mean=float(staleA.mean()), median=float(np.median(staleA))),
            )
            lumG.append(lum_rgb3(final))
            print(f"[{nm}] 接受率 {results[nm]['accept_rate']*100:.2f}% "
                  f"拒绝率 {results[nm]['reject_rate']*100:.2f}% "
                  f"矩阵(geo,col)={st['matrix']} 耗时 {time.time()-t0:.0f}s")

    # 栅格评价
    if "--no-raster" in sys.argv:
        with open(os.path.join(OUT, "summary_partial.json"), "w") as f:
            json.dump(dict(configs=results), f, ensure_ascii=False, indent=1)
        print("跳过栅格评价（--no-raster）")
        return
    odom_frames, odom = load_bag(BAG)
    pose_at = make_interpolator(odom)
    yaml = open(PARAMS).read()
    cam = (float(re.search(r"cam_fx:\s*([\d.]+)", yaml).group(1)),
           float(re.search(r"cam_fy:\s*([\d.]+)", yaml).group(1)),
           float(re.search(r"cam_cx:\s*([\d.]+)", yaml).group(1)),
           float(re.search(r"cam_cy:\s*([\d.]+)", yaml).group(1)),
           int(re.search(r"cam_width:\s*(\d+)", yaml).group(1)),
           int(re.search(r"cam_height:\s*(\d+)", yaml).group(1)),
           np.array([float(v) for v in re.search(r"Rcl:\s*\[([^\]]+)\]", yaml, re.S)
                     .group(1).replace("\n", " ").split(",")]).reshape(3, 3),
           np.array([float(v) for v in re.search(r"Pcl:\s*\[([^\]]+)\]", yaml)
                     .group(1).split(",")]))
    rr = raster_eval(P_el, RGB_el, fr_el, lumA, lumG, n, uniq, odom_frames, pose_at, cam)
    salt = {}
    for k in rr["variants"]:
        salt[k] = dict(pixels=rr["tot"][k], rate=rr["tot"][k] / max(rr["covsum"], 1) * 100,
                       on_multi=rr["tot_multi"][k], on_single=rr["tot_single"][k])
        print(f"椒盐 {k}: {rr['tot'][k]:,} / {rr['covsum']:,} = "
              f"{salt[k]['rate']:.2f}%  (多观测获胜 {rr['tot_multi'][k]:,} / "
              f"单观测 {rr['tot_single'][k]:,})")

    out = dict(obs_total=int(len(vid)), obs_pool=int(len(vid_el)),
               voxels=int(len(uniq)), multi_voxels=int(len(multi)),
               plane_ok_frac=float(plane_ok.mean()), configs=results,
               raster=dict(covsum=rr["covsum"], salt=salt),
               runtime_s=time.time() - t0)
    with open(os.path.join(OUT, "summary.json"), "w") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print(f"完成，用时 {time.time()-t0:.0f}s -> {OUT}/summary.json")


if __name__ == "__main__":
    main()
