#!/usr/bin/env python3
"""控制实验 03：

C1 帧顺序敏感性 —— 合并门槛以"当前代表点"为参照（见 OCTVox_UPDATE_AUDIT.md），
   因此插入顺序会影响最终代表点。对 B/C 分别用 正序 / 反序 / 随机序（seed=0）
   重建地图，报告：代表点位移、位移>1 mm 的子体素比例、局部厚度变化。

C2 扫描密度影响 —— 用 D 组还原每个子体素的逐帧候选数，检查：
   - 每子体素参与帧数、单帧最大候选占比（密度不均衡程度）
   - C 选中代表点所在帧在其子体素候选数分布中的归一化排位
     （0=最少候选帧，1=最多候选帧；无偏应≈0.5）

输出 /home/ywj/slam_data/octvox_ablation_20261110/controls.json
"""
import json
import os
import importlib.util

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = "/home/ywj/slam_data/octvox_ablation_20261110"
BLK, SUB, MAXP = 0.5, 0.25, 20


def _mod(name, fname):
    spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, fname))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


B = _mod("builder", "01_build_maps.py")
E = _mod("eval", "02_evaluate.py")


def load_frames():
    d = np.load(os.path.join(OUT, "inputs.npz"))
    return {"pts": d["pts"], "n": d["n"], "stamps": d["stamps"]}


def load_map(mode):
    d = np.load(os.path.join(OUT, f"map_{mode}.npz"))
    return {k: d[k] for k in d.files}


def thickness(pts, k=12):
    """复算 M1 的局部厚度（与 02_evaluate.m1_thickness 同定义）。"""
    f = E.fit_local_planes(pts.astype(np.float64), k=k)
    gate = (f["plan"] > 0.5) & (f["rmax"] < 0.6) & (f["lam"][:, 1] > 1e-8)
    r = f["resid"][gate]
    if len(r) == 0:
        return dict(n=0)
    return dict(n=int(len(r)), p50=float(np.percentile(r, 50)),
                p90=float(np.percentile(r, 90)),
                rms=float(np.sqrt((r ** 2).mean())))


def keys_of(pts):
    pk, loc, _ = B.subvoxel_keys(np.asarray(pts, dtype=np.float64))
    return pk * 8 + loc


def c1_order(frames, ref):
    n = len(frames["n"])
    rng = np.random.default_rng(0)
    orders = {"reversed": np.arange(n - 1, -1, -1),
              "random": rng.permutation(n)}
    out = {}
    for mode in ("B", "C"):
        k_ref = keys_of(ref[mode]["points"])
        o_ref = np.argsort(k_ref)
        ks = k_ref[o_ref]
        out[mode] = {"time_order": thickness(ref[mode]["points"])}
        for name, order in orders.items():
            m = B.build(mode, frames, list(map(int, order)))
            pts = m["points"]
            disp = np.full(len(ref[mode]["points"]), np.nan)
            k_new = keys_of(pts)
            pos = np.searchsorted(ks, k_new)
            pos = np.clip(pos, 0, len(ks) - 1)
            hit = ks[pos] == k_new
            # 把新地图中命中的代表点对回参考地图下标
            disp_new = np.linalg.norm(
                pts[hit] - ref[mode]["points"][o_ref[pos[hit]]], axis=1)
            out[mode][name] = dict(
                n_sub=len(k_new),
                n_matched=int(hit.sum()),
                changed_gt_1mm_frac=float(np.mean(disp_new > 1e-3)),
                disp_p50_m=float(np.percentile(disp_new, 50)),
                disp_p90_m=float(np.percentile(disp_new, 90)),
                disp_max_m=float(disp_new.max()) if len(disp_new) else None,
                thickness=thickness(pts))
    return out


def c2_density(ref):
    dD = np.load(os.path.join(OUT, "map_D.npz"))
    pts, sub, src = dD["points"].astype(np.float64), dD["sub"], dD["src_frame"]
    o = np.argsort(sub, kind="stable")
    pts, sub, src = pts[o], sub[o], src[o]
    bounds = np.flatnonzero(np.diff(sub)) + 1
    grp = np.split(np.arange(len(sub)), bounds)
    starts = np.r_[0, bounds]
    sub_ids = keys_of(pts)[starts]      # map_D 的 sub 是组号，需还原为真实子体素键

    # C 的 (子体素键 -> 来源帧)
    kC = keys_of(ref["C"]["points"])
    oC = np.argsort(kC)
    kC, srcC = kC[oC], ref["C"]["src_frame"][oC]

    n_frames, maxfrac, rank = [], [], []
    for g, sid in zip(grp, sub_ids):
        if len(g) < 2:
            continue
        fs, cs = np.unique(src[g], return_counts=True)
        n_frames.append(len(fs))
        maxfrac.append(float(cs.max() / cs.sum()))
        j = np.searchsorted(kC, sid)          # kC 已排序? 见下
        if j < len(kC) and kC[j] == sid:
            fi = srcC[j]
            k = np.searchsorted(fs, fi)
            if k < len(fs) and fs[k] == fi:
                rank.append(float((np.sum(cs < cs[k]) + 0.5 * np.sum(cs == cs[k]))
                                  / len(cs)))
    n_frames = np.array(n_frames)
    maxfrac = np.array(maxfrac)
    rank = np.array(rank)
    return dict(
        n_sub_multi=int(len(n_frames)),
        n_frames_per_sub_p50=float(np.median(n_frames)),
        n_frames_per_sub_p90=float(np.percentile(n_frames, 90)),
        n_frames_per_sub_max=int(n_frames.max()),
        max_single_frame_frac_p50=float(np.median(maxfrac)),
        max_single_frame_frac_p90=float(np.percentile(maxfrac, 90)),
        c_src_density_rank_p50=float(np.median(rank)),
        c_src_density_rank_mean=float(rank.mean()),
        c_src_density_rank_p10=float(np.percentile(rank, 10)),
        c_src_density_rank_p90=float(np.percentile(rank, 90)),
        n_rank=int(len(rank)))


def main():
    frames = load_frames()
    ref = {m: load_map(m) for m in ("A", "B", "C", "D")}
    res = {"C1_order_sensitivity": c1_order(frames, ref)}
    res["C2_density_bias"] = c2_density(ref)
    with open(os.path.join(OUT, "controls.json"), "w") as f:
        json.dump(res, f, indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
