#!/bin/bash
python3 - <<'PY'
import sys, re
sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
import numpy as np
from voxel_attribution import load_obs, voxel_table, OBS, VOX, pack16, unpack_vox
from scipy.ndimage import median_filter, convolve, distance_transform_edt

vid, fr, P, RGB, EL = load_obs(OBS)
vid_el, fr_el, P_el, RGB_el = vid[EL], fr[EL], P[EL], RGB[EL]
tb = voxel_table(vid_el, fr_el, P_el, RGB_el)
uniq, inv, n = tb["uniq"], tb["inv"], tb["n"]
L = tb["L"]

order = np.argsort(inv, kind="stable")
inv_s = inv[order]
starts = np.zeros(len(uniq), np.int64)
np.cumsum(n[:-1], out=starts[1:])
rgb_s = RGB_el[order]
multi = np.where(n >= 2)[0]
med = np.zeros((len(uniq), 3)); mean = np.zeros((len(uniq), 3))
for ch in range(3):
    for g in multi:
        seg = rgb_s[starts[g]:starts[g] + n[g], ch]
        med[g, ch] = np.median(seg); mean[g, ch] = seg.mean()
lum_med6 = 0.299*med[:,0] + 0.587*med[:,1] + 0.114*med[:,2]
lum_mean = 0.299*mean[:,0] + 0.587*mean[:,1] + 0.114*mean[:,2]

last_row = order[starts + n - 1]
dm = np.abs(L[last_row] - lum_med6)[multi]
print(f"n>=2 体素 {len(multi):,} ({len(multi)/len(n)*100:.1f}%)  n>=3: {(n>=3).sum():,}  n>=4: {(n>=4).sum():,}")
print(f"  |末次观测lum - med6|: >40 占 {(dm>40).mean()*100:.2f}%  >25 占 {(dm>25).mean()*100:.2f}%  中位 {np.median(dm):.1f}")
d_own = np.abs(lum_med6 - lum_mean)[multi]
print(f"  |med6 - mean| >25 占 {(d_own>25).mean()*100:.2f}%")
lmax = np.array([L[starts[g]:starts[g]+n[g]].max() for g in multi])
lmin = np.array([L[starts[g]:starts[g]+n[g]].min() for g in multi])
rng_ = lmax - lmin
for t in (10, 25, 40, 80):
    print(f"  观测 lum 极差>{t}: {(rng_>t).mean()*100:.2f}% ({int((rng_>t).sum()):,})")

# ---- 栅格域：椒盐像素获胜体素是否多观测（精确 0.1mm 入池，同 replay）----
from blackframe_voxel_replay import load_bag, make_interpolator, read_pcd_xyzrgb, QMEM
odom_frames, odom = load_bag("/home/ywj/slam_data/singleframe/sl_frames")
pose_at = make_interpolator(odom)
Pmap, rgbmap = read_pcd_xyzrgb("/home/ywj/slam_data/singleframe/sl_map/super_livo_visual_a_colored.pcd")
map_keys = pack16(np.round(Pmap * QMEM).astype(np.int32).astype(np.int64) - 0) if False else None
def packq(q, bits):
    a = q.astype(np.int64) + (1 << (bits - 1))
    return (a[:, 0] << (2*bits)) | (a[:, 1] << bits) | a[:, 2]
uniq_map, idx_map = np.unique(packq(np.round(Pmap * QMEM).astype(np.int32), 21), return_index=True)
yaml = open("/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml").read()
fx = float(re.search(r"cam_fx:\s*([\d.]+)", yaml).group(1)); fy = float(re.search(r"cam_fy:\s*([\d.]+)", yaml).group(1))
cx = float(re.search(r"cam_cx:\s*([\d.]+)", yaml).group(1)); cy = float(re.search(r"cam_cy:\s*([\d.]+)", yaml).group(1))
W = int(re.search(r"cam_width:\s*(\d+)", yaml).group(1)); H = int(re.search(r"cam_height:\s*(\d+)", yaml).group(1))
Rcl = np.array([float(v) for v in re.search(r"Rcl:\s*\[([^\]]+)\]", yaml, re.S).group(1).replace("\n", " ").split(",")]).reshape(3, 3)
Pcl = np.array([float(v) for v in re.search(r"Pcl:\s*\[([^\]]+)\]", yaml).group(1).split(",")])

# 融合地图视图：全部入池观测(P_el)从每个相机位姿投影（同 replay 的 A_med6 栅格口径）
allvox = np.floor(P_el / VOX).astype(np.int32)
allg = np.searchsorted(uniq, pack16(allvox))
tot = mlt = covsum = own_cnt = own_multi = 0
for fi, (t, Pb, pk) in enumerate(odom_frames):
    R, tv = pose_at(t)
    Pw = Pb.astype(np.float64) @ R.T + tv
    keys = packq(np.round(Pw * QMEM).astype(np.int32), 21)
    j = np.searchsorted(uniq_map, keys)
    el = uniq_map[np.clip(j, 0, len(uniq_map)-1)] == keys
    Po = Pw[el]
    go = np.searchsorted(uniq, pack16(np.floor(Po / VOX).astype(np.int32)))
    own_cnt += int(el.sum()); own_multi += int((n[go] >= 2).sum())
    p_own = (Po - tv) @ R @ Rcl.T + Pcl
    zo = p_own[:, 2]; mo = zo > 0.3
    uo = fx*p_own[mo, 0]/zo[mo] + cx; vo = fy*p_own[mo, 1]/zo[mo] + cy
    okc = (uo >= 2) & (uo < W-2) & (vo >= 2) & (vo < H-2)
    cov = np.zeros((H, W), bool)
    cov[np.floor(vo[okc]).astype(np.int64), np.floor(uo[okc]).astype(np.int64)] = True
    cc = convolve(cov.astype(np.float32), np.ones((5, 5)), mode="constant") >= 1
    p_cam = (P_el - tv) @ R @ Rcl.T + Pcl
    z = p_cam[:, 2]; m = z > 0.3
    u = fx*p_cam[m, 0]/z[m] + cx; v = fy*p_cam[m, 1]/z[m] + cy
    mm = (u >= 1) & (u < W-1) & (v >= 1) & (v < H-1)
    u, v, zz = u[mm], v[mm], z[m][mm]
    gidx = allg[m][mm]
    imb = n[gidx] >= 2
    im = np.floor(v).astype(np.int64)*W + np.floor(u).astype(np.int64)
    zb = np.full(W*H, np.inf); np.minimum.at(zb, im, zz)
    filled = np.isfinite(zb)
    win = np.full(W*H, -1, np.int64)
    ordz = np.argsort(-zz)
    win[im[ordz]] = np.arange(len(im))[ordz]
    Lpix = np.zeros(W*H); mpix = np.zeros(W*H, bool)
    wi = win[filled]
    Lpix[filled] = lum_med6[gidx[wi]]
    mpix[filled] = imb[wi]
    Limg = Lpix.reshape(H, W); Mimg = mpix.reshape(H, W)
    _, near = distance_transform_edt(~filled.reshape(H, W), return_indices=True)
    Lf = Limg[near[0], near[1]]; Mf = Mimg[near[0], near[1]]
    jump = (np.abs(Lf - median_filter(Lf, 5, mode="nearest")) > 40) & cc
    tot += int(jump.sum()); mlt += int((jump & Mf).sum()); covsum += int(cc.sum())
    if fi < 3:
        print(f"  dbg 帧#{fi}: 当前帧入池{int(el.sum())} 掩码{int(cc.sum())} 椒盐{int(jump.sum())}")
print(f"\n融合地图视图(全部帧): 椒盐像素 {tot:,} / 掩码 {covsum:,} = {tot/max(covsum,1)*100:.2f}%")
print(f"  获胜体素为多观测(n>=2) 的椒盐像素: {mlt:,} ({mlt/max(tot,1)*100:.1f}%)")
print(f"  像素域参照: 当前帧入池点落在多观测体素的比例 = {own_multi/max(own_cnt,1)*100:.2f}%")
PY
