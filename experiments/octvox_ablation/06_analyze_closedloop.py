#!/usr/bin/env python3
"""06：解析阶段二闭环实验的 debug.csv，输出 A/B/C 三组运行指标。

debug.csv 由 lio.debug.level=2 写出（见 super_lio.cpp），每帧一行：
  t, ptsize, n_knn_fail, n_plane_fail, n_res_fail, n_effect,
  fit_res_mean, fit_res_p95, iters, dx_inf, cond_yk, qk_finite,
  htvh_ev_min, htvh_ev_max, px,py,pz, vx,vy,vz, rvx,rvy,rvz, imu_scale,
  rvx_pred,rvy_pred,rvz_pred, bg*, ba*, g*, dt_imu_end, frame_dt,
  deg_cond, deg_axes, deg_cond_t, deg_evt_min, deg_dir_v, deg_dir_g, deg_dir_z

输出：
  closedloop_metrics.csv    每组一行（含 CPU/内存）
  closedloop_per_frame.csv  逐帧合并表（含三组在公共时间戳上的插值位置差）
  closedloop_summary.json
"""
import json
import os
import re

import numpy as np

OUT = "/home/ywj/slam_data/octvox_closedloop_20261110"
STAGE1_WINDOW = (946685507.0, 946685531.5)
RUNS = {"A": 0, "B": 2, "C": 1}
COLS = ["t", "ptsize", "n_knn_fail", "n_plane_fail", "n_res_fail", "n_effect",
        "fit_res_mean", "fit_res_p95", "iters", "dx_inf", "cond_yk", "qk_finite",
        "ev_min", "ev_max", "px", "py", "pz", "vx", "vy", "vz", "rvx", "rvy", "rvz",
        "imu_scale", "rvx_pred", "rvy_pred", "rvz_pred", "bgx", "bgy", "bgz",
        "bax", "bay", "baz", "gx", "gy", "gz", "dt_imu_end", "frame_dt",
        "deg_cond", "deg_axes", "deg_cond_t", "deg_evt_min",
        "deg_dir_v", "deg_dir_g", "deg_dir_z"]


def load_debug(path):
    a = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return {c: a[:, i] for i, c in enumerate(COLS)}


def parse_time(path):
    txt = open(path, errors="ignore").read()
    def g(k):
        m = re.search(rf"{re.escape(k)}:\s*([0-9:.]+)", txt)
        return m.group(1) if m else None
    rss = re.search(r"Maximum resident set size \(kbytes\):\s*(\d+)", txt)
    return dict(wall=g("Elapsed (wall clock) time (h:mm:ss or m:ss)"),
                user=g("User time (seconds)"), sys=g("System time (seconds)"),
                cpu_pct=g("Percent of CPU this job got"),
                max_rss_mb=(int(rss.group(1)) / 1024.0 if rss else None))


def path_len(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def main():
    import sys
    full = "--full" in sys.argv          # --full：用整段公共时间范围（含初始化与结束段）
    data = {}
    for k in RUNS:
        p = os.path.join(OUT, f"rep_{k}", "debug.csv")
        if not os.path.exists(p):
            raise SystemExit(f"missing {p}")
        data[k] = load_debug(p)
        print(f"{k}: {len(data[k]['t'])} frames, t=[{data[k]['t'][0]:.2f}, "
              f"{data[k]['t'][-1]:.2f}]")

    # 统一分析窗口：优先阶段一窗口；若时间基准不同则退回到三组公共时间范围
    lo = max(d["t"][0] for d in data.values())
    hi = min(d["t"][-1] for d in data.values())
    if full:
        w0, w1, wtag = lo, hi, "common_overlap_full"
    elif data["A"]["t"][0] <= STAGE1_WINDOW[0] and data["A"]["t"][-1] >= STAGE1_WINDOW[1]:
        w0, w1, wtag = STAGE1_WINDOW[0], STAGE1_WINDOW[1], "stage1_window"
    else:
        w0, w1, wtag = lo, hi, "common_overlap"
    print(f"analysis window ({wtag}): {w0:.2f} .. {w1:.2f}")

    rows, per_frame = [], {}
    for k, d in data.items():
        m = (d["t"] >= w0) & (d["t"] <= w1)
        dd = {c: v[m] for c, v in d.items()}
        p = np.stack([dd["px"], dd["py"], dd["pz"]], 1)
        steps = np.linalg.norm(np.diff(p, axis=0), axis=1)
        dt = np.diff(dd["t"])
        fdt = dd["frame_dt"]
        med_dt = float(np.median(fdt[fdt > 0])) if np.any(fdt > 0) else float("nan")
        gaps = dt[dt > 1.6 * med_dt] if len(dt) else np.array([])
        corr = np.degrees(np.linalg.norm(
            np.stack([dd["rvx"] - dd["rvx_pred"],
                      dd["rvy"] - dd["rvy_pred"],
                      dd["rvz"] - dd["rvz_pred"]], 1), axis=1))
        resid = dd["fit_res_mean"][dd["fit_res_mean"] > 0]
        match = dd["n_effect"] / np.maximum(dd["ptsize"], 1)
        r = dict(run=k, rep_mode=RUNS[k], n_frames=int(m.sum()),
                 t0=float(w0), t1=float(w1),
                 path_len_m=path_len(p),
                 step_p50_mm=float(np.percentile(steps, 50) * 1e3) if len(steps) else None,
                 step_p90_mm=float(np.percentile(steps, 90) * 1e3) if len(steps) else None,
                 step_max_mm=float(steps.max() * 1e3) if len(steps) else None,
                 frame_dt_p50_ms=float(med_dt * 1e3),
                 frame_dt_p99_ms=float(np.percentile(fdt, 99) * 1e3),
                 n_gaps=int(len(gaps)), gap_total_ms=float(gaps.sum() * 1e3) if len(gaps) else 0.0,
                 resid_mean_p50_mm=float(np.percentile(resid, 50) * 1e3) if len(resid) else None,
                 resid_mean_p90_mm=float(np.percentile(resid, 90) * 1e3) if len(resid) else None,
                 match_ratio_p50=float(np.percentile(match, 50)),
                 knn_fail_frac=float(np.mean(dd["n_knn_fail"] / np.maximum(dd["ptsize"], 1))),
                 plane_fail_frac=float(np.mean(dd["n_plane_fail"] / np.maximum(dd["ptsize"], 1))),
                 att_corr_p50_deg=float(np.percentile(corr, 50)),
                 att_corr_p90_deg=float(np.percentile(corr, 90)),
                 dx_inf_p50=float(np.percentile(dd["dx_inf"], 50)),
                 deg_cond_p50=float(np.percentile(dd["deg_cond"], 50)),
                 deg_axes_mean=float(np.mean(dd["deg_axes"])),
                 gz_mean=float(np.mean(dd["gx"] ** 2 + dd["gy"] ** 2 + dd["gz"] ** 2) ** 0.5))
        r.update(parse_time(os.path.join(OUT, f"rep_{k}", "time.txt")))
        rows.append(r)
        # per_frame 保留逐帧原始残差（与 t 等长），便于按时间戳对齐比较
        per_frame[k] = dict(t=dd["t"], p=p, resid=dd["fit_res_mean"].copy(),
                            match=match, corr=corr, dx=dd["dx_inf"])

    # 三组在同一时间网格上的轨迹差（无 GT，仅相对一致性）
    tref = per_frame["A"]["t"]
    div = {}
    for k in ("B", "C"):
        pi = np.stack([np.interp(tref, per_frame[k]["t"], per_frame[k]["p"][:, i])
                       for i in range(3)], 1)
        d = np.linalg.norm(per_frame["A"]["p"] - pi, axis=1)
        div[f"{k}-A"] = dict(p50_mm=float(np.percentile(d, 50) * 1e3),
                             p90_mm=float(np.percentile(d, 90) * 1e3),
                             max_mm=float(d.max() * 1e3),
                             final_mm=float(d[-1] * 1e3))

    summary = dict(window=dict(t0=w0, t1=w1, tag=wtag),
                   runs=rows, trajectory_divergence=div)

    # 配对检定：同一时间戳（最近邻对齐）下的逐帧残差/匹配率差，A 为基线
    from scipy.stats import wilcoxon
    rng_t = per_frame["A"]["t"]
    paired = {}
    for k in ("B", "C"):
        jj = np.array([int(np.argmin(np.abs(per_frame[k]["t"] - t))) for t in rng_t])
        dres = per_frame[k]["resid"][jj] - per_frame["A"]["resid"]
        dmatch = per_frame[k]["match"][jj] - per_frame["A"]["match"]
        ok = (per_frame[k]["resid"][jj] > 0) & (per_frame["A"]["resid"] > 0)
        p1 = float(wilcoxon(dres[ok]).pvalue) if ok.sum() > 10 else None
        p2 = float(wilcoxon(dmatch).pvalue)
        paired[f"{k}-A"] = dict(
            n_resid_pair=int(ok.sum()),
            dresid_p50_mm=float(np.percentile(dres[ok], 50) * 1e3),
            dresid_p90_mm=float(np.percentile(dres[ok], 90) * 1e3),
            dresid_wilcoxon_p=p1,
            dmatch_p50=float(np.percentile(dmatch, 50)),
            dmatch_wilcoxon_p=p2)
    summary["paired_vs_A"] = paired
    suffix = "_full" if full else ""
    with open(os.path.join(OUT, f"closedloop_summary{suffix}.json"), "w") as f:
        json.dump(summary, f, indent=1)

    import csv
    with open(os.path.join(OUT, f"closedloop_metrics{suffix}.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

    pf = []
    for i, t in enumerate(tref):
        rr = dict(t=float(t))
        for k in ("A", "B", "C"):
            d = per_frame[k]
            j = int(np.argmin(np.abs(d["t"] - t)))
            rr[f"{k}_resid_mm"] = float(d["resid"][j] * 1e3) if j < len(d["resid"]) else None
            rr[f"{k}_match"] = float(d["match"][j])
            rr[f"{k}_att_corr_deg"] = float(d["corr"][j])
            rr[f"{k}_px"], rr[f"{k}_py"], rr[f"{k}_pz"] = [float(x) for x in d["p"][j]]
        pf.append(rr)
    with open(os.path.join(OUT, f"closedloop_per_frame{suffix}.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(pf[0].keys()))
        w.writeheader(); w.writerows(pf)

    for r in rows:
        print(json.dumps(r, ensure_ascii=False))
    print(json.dumps(div, indent=1))


if __name__ == "__main__":
    main()
