#!/usr/bin/env python3
"""汇总 latency_probe.py 的 CSV，输出各臂 odom 输出节拍与整帧延时。

   python3 tools/latency_summary.py --label FL2 run/fl2/latency.csv \
       --label A run/a/latency.csv --label B run/b/latency.csv

也可直接传 run 目录（自动找 latency.csv），或 --json 输出机器可读结果。
配对规则：每条 odom 按其 header.stamp 找最近邻雷达帧 header.stamp
（容差 --tol-ms，默认 250 ms；FL2 的 stamp 为帧尾，差值 ≈ 一个扫描周期）。
"""
import argparse
import bisect
import csv
import json
import os
import statistics as st


def load(path):
    lidar, odom = [], []
    with open(path, newline="", errors="ignore") as f:
        for row in csv.reader(f):
            if len(row) != 3 or row[0] not in ("lidar", "odom"):
                continue
            recv, stamp = int(row[1]), int(row[2])
            (lidar if row[0] == "lidar" else odom).append((stamp, recv))
    lidar.sort()
    odom.sort()
    return lidar, odom


def pct(vals, p):
    if not vals:
        return float("nan")
    s = sorted(vals)
    k = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[k]


def periods_ms(stamps):
    return [(b - a) / 1e6 for a, b in zip(stamps, stamps[1:])]


def summarize(lidar, odom, tol_ms):
    l_stamps = [s for s, _ in lidar]
    l_recv = {s: r for s, r in lidar}
    algo, e2e, off, matched = [], [], [], 0
    tol_ns = tol_ms * 1e6
    for st_, recv in odom:
        i = bisect.bisect_left(l_stamps, st_)
        cand = []
        if i < len(l_stamps):
            cand.append(l_stamps[i])
        if i > 0:
            cand.append(l_stamps[i - 1])
        if not cand:
            continue
        near = min(cand, key=lambda x: abs(x - st_))
        if abs(near - st_) > tol_ns:
            continue
        matched += 1
        algo.append((recv - l_recv[near]) / 1e6)
        e2e.append((recv - near) / 1e6)
        off.append((st_ - near) / 1e6)

    op = periods_ms([s for s, _ in odom])
    lp = periods_ms(l_stamps)
    # 有些 bag 的 header.stamp 走设备时钟（如 move_livoxclk 的 2020 纪元），
    # 与墙钟不在同一时基，此时 e2e / transport 无意义，只保留 algo 与节拍。
    clock_ok = True
    if e2e:
        med_e2e = pct(e2e, 50)
        if abs(med_e2e) > 60_000.0:
            clock_ok = False
    e2e_med = pct(e2e, 50) if clock_ok else float("nan")
    e2e_p95 = pct(e2e, 95) if clock_ok else float("nan")
    trans_med = pct([(r - s) / 1e6 for s, r in lidar], 50) if clock_ok else float("nan")
    out = {
        "clock_ok": clock_ok,
        "n_lidar": len(lidar),
        "n_odom": len(odom),
        "matched": matched,
        "match_ratio": matched / len(odom) if odom else 0.0,
        "lidar_period_ms_med": pct(lp, 50),
        "lidar_gaps_gt150": sum(1 for x in lp if x > 150),
        "odom_period_ms_med": pct(op, 50),
        "odom_period_ms_p95": pct(op, 95),
        "odom_period_ms_max": max(op) if op else float("nan"),
        "odom_gaps_gt150": sum(1 for x in op if x > 150),
        "odom_span_s": (odom[-1][0] - odom[0][0]) / 1e9 if len(odom) > 1 else 0.0,
        "stamp_offset_ms_med": pct(off, 50),
        "algo_ms_med": pct(algo, 50),
        "algo_ms_p95": pct(algo, 95),
        "algo_ms_max": max(algo) if algo else float("nan"),
        "e2e_ms_med": e2e_med,
        "e2e_ms_p95": e2e_p95,
        "transport_ms_med": trans_med,
        "algo_ms": algo,
        "e2e_ms": e2e,
        "odom_period_ms": op,
    }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", action="append", default=[])
    ap.add_argument("--tol-ms", type=float, default=250.0)
    ap.add_argument("--json", default="")
    ap.add_argument("paths", nargs="+")
    a = ap.parse_args()
    labels = a.label or [os.path.basename(os.path.dirname(p.rstrip("/"))) for p in a.paths]
    rows = []
    for lab, p in zip(labels, a.paths):
        if os.path.isdir(p):
            p = os.path.join(p, "latency.csv")
        if not os.path.exists(p):
            print(f"[skip] {lab}: 缺 {p}")
            continue
        lidar, odom = load(p)
        s = summarize(lidar, odom, a.tol_ms)
        s["label"] = lab
        s["csv"] = p
        rows.append(s)

    hdr = (f"{'arm':16s}{'odom':>6s}{'span_s':>8s}"
           f"{'period_med':>11s}{'p95':>8s}{'max':>8s}{'gap>150':>8s}"
           f"{'stamp_off':>10s}{'algo_med':>9s}{'p95':>8s}{'max':>8s}{'e2e_med':>9s}")
    print(hdr)
    print("-" * len(hdr))
    for s in rows:
        print(f"{s['label']:16s}{s['n_odom']:>6d}{s['odom_span_s']:>8.1f}"
              f"{s['odom_period_ms_med']:>11.2f}{s['odom_period_ms_p95']:>8.2f}"
              f"{s['odom_period_ms_max']:>8.1f}{s['odom_gaps_gt150']:>8d}"
              f"{s['stamp_offset_ms_med']:>10.1f}{s['algo_ms_med']:>9.2f}"
              f"{s['algo_ms_p95']:>8.2f}{s['algo_ms_max']:>8.1f}{s['e2e_ms_med']:>9.2f}")
    print("-" * len(hdr))
    print("单位 ms；algo=odom到达−雷达帧到达（跨臂可比），e2e=odom到达−雷达stamp；"
          "stamp_off=odom.stamp−雷达.stamp（FL2≈+100 ms 属帧尾口径）。")
    if a.json:
        slim = [{k: v for k, v in s.items() if k not in ("algo_ms", "e2e_ms", "odom_period_ms")}
                for s in rows]
        with open(a.json, "w") as f:
            json.dump(slim, f, indent=2, ensure_ascii=False)
        print(f"[json] {a.json}")


if __name__ == "__main__":
    main()
