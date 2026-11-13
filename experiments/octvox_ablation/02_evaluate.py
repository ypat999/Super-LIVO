#!/usr/bin/env python3
"""阶段一 02：A/B/C/D 地图几何评价。

指标（全部给出定义与单位）：
  M1 地图局部厚度   leave-one-out 邻域平面，残差 |n·(p-c)|，单位 m
  M2 留出帧残差     时间分块留出建图 → 测试帧点到面残差（主指标）
  M3 结构边缘保真   以 D 全量观测为参考：edge_dist / edge_cover / off5cm（见 m3_structure）
  M4 输入一致性     D 组每子体素内多帧观测离散度（与策略无关的输入基线）
  M5 中心偏置       §5.4：代表点到子体素中心距离、沿局部法向的组间位移

输出：/home/ywj/slam_data/octvox_ablation_20261110/
  metrics_M1.csv, metrics_M2_<block>s.csv, per_frame_residual_<block>s.csv,
  metrics_M3.csv, metrics_M4.json, metrics_M5.json, summary.json
"""
import argparse
import csv
import json
import os
import sys
import importlib.util

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.environ.get("OCTVOX_OUT", "/home/ywj/slam_data/octvox_ablation_20261110")
BLK = 0.5          # 父体素
SUB = 0.25         # 子体素
MERGE = 0.1
MAXP = 20

spec = importlib.util.spec_from_file_location(
    "builder", os.path.join(HERE, "01_build_maps.py"))
B = importlib.util.module_from_spec(spec)
spec.loader.exec_module(B)


def load_frames():
    d = np.load(os.path.join(OUT, "inputs.npz"))
    return {"pts": d["pts"], "n": d["n"], "stamps": d["stamps"]}


def load_map(mode):
    d = np.load(os.path.join(OUT, f"map_{mode}.npz"))
    return {k: d[k] for k in d.files}


def fit_local_planes(pts, k=12):
    """对每个点：用其 k-1 个近邻（不含自身）拟合平面。返回法向/平面点/残差/特征值。"""
    tree = cKDTree(pts)
    d, idx = tree.query(pts, k=k)
    nb = pts[idx[:, 1:]]
    c = nb.mean(axis=1)
    q = nb - c[:, None, :]
    cov = np.einsum("nki,nkj->nij", q, q) / nb.shape[1]
    w, v = np.linalg.eigh(cov)
    n = v[:, :, 0]
    resid = np.abs(np.einsum("ni,ni->n", pts - c, n))
    sres = np.einsum("ni,ni->n", pts - c, n)
    plan = (w[:, 1] - w[:, 0]) / np.maximum(w[:, 2], 1e-12)
    return dict(n=n, c=c, resid=resid, sres=sres, lam=w, plan=plan, rmax=d[:, -1], tree=tree)


def pct(x):
    if len(x) == 0:
        return dict(n=0)
    x = np.asarray(x, dtype=np.float64)
    return dict(n=int(len(x)), p50=float(np.percentile(x, 50)),
                p90=float(np.percentile(x, 90)), p95=float(np.percentile(x, 95)),
                rms=float(np.sqrt((x ** 2).mean())), mean=float(x.mean()))


def m1_thickness(maps):
    rows = []
    for mode in ("A", "B", "C", "D"):
        p = maps[mode]["points"].astype(np.float64)
        if mode == "D":                      # D 全量点：随机抽样以控成本
            rng = np.random.default_rng(0)
            sel = rng.choice(len(p), size=min(40000, len(p)), replace=False)
            p = p[sel]
        f = fit_local_planes(p)
        gate = (f["plan"] > 0.5) & (f["rmax"] < 0.6) & (f["lam"][:, 1] > 1e-8)
        r = f["resid"][gate]
        st = pct(r)
        st.update(mode=mode, n_total=int(len(p)), gate_frac=float(gate.mean()))
        rows.append(st)
        print(f"[M1] {mode}: gate={gate.mean()*100:.1f}% "
              f"P50={st['p50']*1000:.1f}mm P90={st['p90']*1000:.1f}mm")
    return rows


def build_holdout(mode, frames, block_ids, keep):
    if mode == "D":                      # D 不降采样：直接拼接保留区块的全部观测点
        pts = [frames["pts"][fi][:frames["n"][fi]] for fi in range(len(frames["n"]))
               if block_ids[fi] not in keep]
        pts = np.concatenate(pts).astype(np.float64)
        return dict(points=pts, n_sub=-1, n_parent=-1)
    mb = B.MapBuilder(mode)
    for fi in range(len(frames["n"])):
        if block_ids[fi] in keep:
            continue
        mb.add_frame(frames["pts"][fi][:frames["n"][fi]], fi)
    return mb.export()


def eval_cloud(test, mpts, k=8, rad=0.8, tree=None):
    tree = tree if tree is not None else cKDTree(mpts)
    d, idx = tree.query(test, k=k)
    nb = mpts[idx]
    c = nb.mean(axis=1)
    q = nb - c[:, None, :]
    cov = np.einsum("nki,nkj->nij", q, q) / k
    w, v = np.linalg.eigh(cov)
    n = v[:, :, 0]
    resid = np.abs(np.einsum("ni,ni->n", test - c, n))
    plan = (w[:, 1] - w[:, 0]) / np.maximum(w[:, 2], 1e-12)
    valid = (d[:, -1] < rad) & (plan > 0.5) & (w[:, 1] > 1e-8)
    return resid, valid


def m2_holdout(maps, frames, block_s, modes=("A", "B", "C", "D"),
               paired=("A", "B", "C")):
    """时间分块留出：留出一个连续时间块的建图，用该块帧做测试。

    同时报告：
      marginal  每组各自有效点上统计（有效点集不同，仅作参考）
      paired    A/B/C 同时有效的测试点（同一批点、同一帧，配对可比，主结论用这个）
    D 的密度与其他组不同（每子体素 1~114 观测），其有效点集与 A/B/C 差异极大，
    纳入配对交集会把样本压缩约 7 倍并引入选择偏差，故 D 只报 marginal。
    """
    t0 = frames["stamps"][0]
    bid = np.floor((frames["stamps"] - t0) / block_s).astype(int)
    perframe = []
    acc = {m: [] for m in modes}
    accp = {m: [] for m in modes}
    for b in np.unique(bid):
        hold = {m: build_holdout(m, frames, bid, set([b])) for m in modes}
        pts = {m: hold[m]["points"].astype(np.float64) for m in modes}
        trees = {m: cKDTree(pts[m]) for m in modes}
        for fi in np.where(bid == b)[0]:
            test = frames["pts"][fi][:frames["n"][fi]].astype(np.float64)
            res, oks = {}, {}
            for m in modes:
                r, ok = eval_cloud(test, pts[m], tree=trees[m])
                res[m], oks[m] = r, ok
                acc[m].append(r[ok])
            common = np.logical_and.reduce([oks[m] for m in paired])
            for m in modes:
                if m in paired:
                    accp[m].append(res[m][common])
                rv = res[m][oks[m]]
                rvc = res[m][common]
                perframe.append(dict(mode=m, block=int(b), frame=int(fi),
                                     stamp=float(frames["stamps"][fi]),
                                     n_point=int(len(test)), n_valid=int(oks[m].sum()),
                                     valid_frac=float(oks[m].sum() / max(len(test), 1)),
                                     n_common=int(common.sum()),
                                     p50=float(np.percentile(rv, 50)) if len(rv) else None,
                                     p90=float(np.percentile(rv, 90)) if len(rv) else None,
                                     rms=float(np.sqrt((rv ** 2).mean())) if len(rv) else None,
                                     paired_p50=float(np.percentile(rvc, 50)) if len(rvc) else None,
                                     paired_rms=(float(np.sqrt((rvc ** 2).mean()))
                                                 if len(rvc) else None)))
        print(f"[M2 {block_s}s] block{b} maps=" +
              " ".join(f"{m}:{len(pts[m])}" for m in modes))
    rows = []
    for m in modes:
        allr = np.concatenate(acc[m]) if acc[m] else np.array([])
        pr = np.concatenate(accp[m]) if accp[m] else np.array([])
        st = pct(allr)
        st.update(mode=m, block_s=block_s, n_paired=int(len(pr)),
                  paired_p50=float(np.percentile(pr, 50)) if len(pr) else None,
                  paired_p90=float(np.percentile(pr, 90)) if len(pr) else None,
                  paired_rms=float(np.sqrt((pr ** 2).mean())) if len(pr) else None)
        rows.append(st)
        extra = (f"| paired P50={st['paired_p50']*1000:.2f}mm "
                 f"RMS={st['paired_rms']*1000:.2f}mm n={len(pr)}") if len(pr) else ""
        print(f"[M2 {block_s}s] {m}: marginal P50={st.get('p50', 0)*1000:.2f}mm "
              f"n={st.get('n', 0)} {extra}")
    return rows, perframe


def m4_input_agreement():
    d = np.load(os.path.join(OUT, "map_D.npz"))
    pts, sub = d["points"].astype(np.float64), d["sub"]
    order = np.argsort(sub, kind="stable")
    pts, sub = pts[order], sub[order]
    bounds = np.flatnonzero(np.diff(sub)) + 1
    groups = np.split(pts, bounds)
    spread, ns = [], []
    for g in groups:
        if len(g) < 2:
            continue
        c = g.mean(axis=0)
        spread.append(float(np.sqrt(((g - c) ** 2).sum(1).mean())))
        ns.append(len(g))
    spread = np.array(spread)
    st = pct(spread)
    st.update(n_obs_p50=float(np.median(ns)), n_obs_p90=float(np.percentile(ns, 90)),
              frac_multi=float(np.mean(np.array(ns) >= 2)))
    return st


def m5_center_bias(maps):
    """匹配同一子体素键，比较 A/B/C 代表点位置与到中心距离。"""
    key = {}
    for mode in ("A", "B", "C"):
        p = maps[mode]["points"].astype(np.float64)
        pk, loc, ctr = B.subvoxel_keys(p)
        key[mode] = (pk * 8 + loc, p, ctr, maps[mode]["d_center"])
    common = np.intersect1d(np.intersect1d(key["A"][0], key["B"][0]), key["C"][0])
    out = []
    idx = {}
    for mode in ("A", "B", "C"):
        s = key[mode][0]
        o = np.argsort(s)
        pos = np.searchsorted(s[o], common)
        idx[mode] = o[pos]
    dA = key["A"][3][idx["A"]]
    dB = key["B"][3][idx["B"]]
    dC = key["C"][3][idx["C"]]
    pA, pB, pC = key["A"][1][idx["A"]], key["B"][1][idx["B"]], key["C"][1][idx["C"]]
    # 用 A 的最终代表点邻域拟合局部法向 → 沿法向的组间位移（近似）
    f = fit_local_planes(pA)
    n = f["n"]
    dBC = np.einsum("ni,ni->n", pC - pB, n)
    dAB = np.einsum("ni,ni->n", pA - pB, n)
    out.append(dict(cmp="d_center A/B/C",
                    p50=[float(np.percentile(x, 50)) for x in (dA, dB, dC)],
                    p90=[float(np.percentile(x, 90)) for x in (dA, dB, dC)]))
    out.append(dict(cmp="|pC-pB| along n_A", **{k: v for k, v in pct(np.abs(dBC)).items()}))
    out.append(dict(cmp="|pA-pB| along n_A", **{k: v for k, v in pct(np.abs(dAB)).items()}))
    out.append(dict(cmp="|pC-pB| 3D", **pct(np.linalg.norm(pC - pB, axis=1))))
    out.append(dict(cmp="|pA-pB| 3D", **pct(np.linalg.norm(pA - pB, axis=1))))
    out.append(dict(cmp="n_common_subvoxels", n=int(len(common))))
    return out


def m3_structure(maps, k=20, curv_thr=0.5):
    """结构边缘保真度 —— 以 D 全量观测云作为几何参考。

    定义（单位 m）：
      edge_dist   A/B/C 每个地图点到最近"参考边缘点"的距离
      edge_cover  每个参考边缘点到最近 A/B/C 地图点的距离（大 ⇒ 缺口/边缘缺失）
      off5cm      地图点到最近参考表面点距离 > 5 cm 的比例（虚假点/双层扩散代理）
    参考边缘点 = D 云（降采样到 0.1 m 以消除观测密度效应）中 k 近邻 PCA 的
                 λ0/λ1 > curv_thr（取曲率最高约 10% 分位）的点。
    表面距离参考云 = D 降采样到 0.05 m（网格间隙 ~3.5 cm，计入基线）。
    """
    pD = maps["D"]["points"].astype(np.float64)
    pDs10 = B.voxel_closest(pD, 0.1)
    pDs05 = B.voxel_closest(pD, 0.05)
    fD = fit_local_planes(pDs10, k=k)
    curv = fD["lam"][:, 0] / np.maximum(fD["lam"][:, 1], 1e-12)
    edge = pDs10[(curv > curv_thr) & (fD["lam"][:, 0] > 1e-8)]
    treeE, treeS = cKDTree(edge), cKDTree(pDs05)
    rows = []
    for mode in ("A", "B", "C"):
        p = maps[mode]["points"].astype(np.float64)
        d_edge, _ = treeE.query(p)
        d_surf, _ = treeS.query(p)
        d_cov, _ = cKDTree(p).query(edge)
        st = dict(mode=mode, n_map=len(p), n_edge_ref=len(edge))
        st.update({f"edge_dist_{q}": float(np.percentile(d_edge, float(q[1:])))
                   for q in ("p50", "p90", "p95")})
        st.update({f"edge_cover_{q}": float(np.percentile(d_cov, float(q[-2:])))
                   for q in ("p50", "p90", "p95")})
        st["off5cm_frac"] = float(np.mean(d_surf > 0.05))
        st["off_surface_p90"] = float(np.percentile(d_surf, 90))
        rows.append(st)
        print(f"[M3] {mode}: edge_dist_p50={st['edge_dist_p50']*1000:.1f}mm "
              f"edge_cover_p50={st['edge_cover_p50']*1000:.1f}mm "
              f"off5cm={st['off5cm_frac']*100:.1f}%")
    return rows


def paired_frame_test(pf, base="A"):
    """逐帧配对检验：以帧为独立单位（每帧内部点高度相关，不可当作独立样本）。

    差异定义 = 该策略的配对 RMS − A 的配对 RMS（m）。负值表示该策略更好。
    """
    from scipy.stats import wilcoxon
    by = {}
    for r in pf:
        by.setdefault(r["frame"], {})[r["mode"]] = r
    out = {}
    for m in ("B", "C"):
        d = [dd[m]["paired_rms"] - dd[base]["paired_rms"] for dd in by.values()
             if m in dd and base in dd and dd[m]["paired_rms"] is not None
             and dd[base]["paired_rms"] is not None]
        if len(d) < 10:
            continue
        d = np.array(d)
        p = float(wilcoxon(d).pvalue) if np.any(d != 0) else None
        out[f"{m}-{base}"] = dict(n_frames=int(len(d)),
                                  rms_diff_median_m=float(np.median(d)),
                                  rms_diff_mean_m=float(np.mean(d)),
                                  frac_frames_better=float(np.mean(d < 0)),
                                  wilcoxon_p=p)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--blocks", default="5,2")
    ap.add_argument("--only", default="M1,M2,M3,M4,M5",
                    help="逗号分隔，选择要计算的指标（其余沿用已有 summary.json）")
    a = ap.parse_args()
    sel = set(x.strip().upper() for x in a.only.split(","))
    frames = load_frames()
    maps = {m: load_map(m) for m in ("A", "B", "C", "D")}
    sp = os.path.join(OUT, "summary.json")
    summary = json.load(open(sp)) if os.path.exists(sp) else {}

    if "M1" in sel:
        r1 = m1_thickness(maps)
        summary["M1"] = r1
        with open(os.path.join(OUT, "metrics_M1.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(r1[0].keys()))
            w.writeheader(); w.writerows(r1)

    if "M2" in sel:
        r2 = {}
        for bs in [float(x) for x in a.blocks.split(",")]:
            rows, pf = m2_holdout(maps, frames, bs)
            r2[str(bs)] = rows
            with open(os.path.join(OUT, f"metrics_M2_{bs:g}s.csv"), "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader(); w.writerows(rows)
            with open(os.path.join(OUT, f"per_frame_residual_{bs:g}s.csv"), "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(pf[0].keys()))
                w.writeheader(); w.writerows(pf)
            summary.setdefault("M2_paired_test", {})[f"{bs:g}s"] = paired_frame_test(pf)
            print(f"[M2 {bs:g}s] paired frame test: " +
                  json.dumps(summary["M2_paired_test"][f"{bs:g}s"]))
        summary["M2"] = r2

    if "M3" in sel:
        r3 = m3_structure(maps)
        summary["M3"] = r3
        with open(os.path.join(OUT, "metrics_M3.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(r3[0].keys()))
            w.writeheader(); w.writerows(r3)

    if "M4" in sel:
        r4 = m4_input_agreement()
        summary["M4"] = r4
        with open(os.path.join(OUT, "metrics_M4.json"), "w") as f:
            json.dump(r4, f, indent=1)

    if "M5" in sel:
        r5 = m5_center_bias(maps)
        summary["M5"] = r5
        with open(os.path.join(OUT, "metrics_M5.json"), "w") as f:
            json.dump(r5, f, indent=1)

    with open(sp, "w") as f:
        json.dump(summary, f, indent=1)
    print(json.dumps(summary.get("M3"), indent=1))


if __name__ == "__main__":
    main()
