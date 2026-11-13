#!/usr/bin/env python3
"""外部参考对照：把官方 FAST-LIVO2 的轨迹当作「第三方参考」（不是真值）。

三件事，分开报告，互不替代：
  (1) 重复性：参考系统自己跑两遍，在分析窗内的互差。这是「参考能否当真值」的上界。
  (2) 偏离量：A/B/C 闭环轨迹与参考轨迹在 SE(3) 最优拟合后的逐帧平移差。
  (3) 干净段：剔除参考自身瞬态跳变帧后的同类统计。

说明：只说明「一致/不一致」，不说明「谁准」。且 A 与参考同源（同一 LIO 家族），
见 docs/MAP_UPDATE_CLOSED_LOOP.md §6。

输入：
  /home/ywj/slam_data/fastlivo2_ref_20261110/ref{1,2}/bag   /aft_mapped_to_init
  /home/ywj/slam_data/fastlivo2_ref_20261110/ref{1,2}/node.log
  /home/ywj/slam_data/octvox_closedloop_20261110/rep_{A,B,C}/debug.csv
输出：
  /home/ywj/slam_data/fastlivo2_ref_20261110/external_comparison.json
"""
import json
import os
import re

import numpy as np
from scipy.spatial.transform import Rotation as Rot

REF_ROOT = "/home/ywj/slam_data/fastlivo2_ref_20261110"
CL_ROOT = "/home/ywj/slam_data/octvox_closedloop_20261110"
OUT = REF_ROOT
WINDOW = (946685507.0, 946685531.5)
TOL = 6e-3
JUMP = 0.3          # 单帧平移超过此值视为瞬态跳变（0.1 s -> >3 m/s，本场景不可能）

# FAST-LIVO2 的 /aft_mapped_to_init.header.stamp = node clock now()（wall clock），
# 源码 LIVMapper.cpp L1561 把 fromSec(last_timestamp_lidar) 注释掉了。
# 但 node.log 每帧打印 "Get LiDAR, its header time: <lidar_stamp>"，带 wall 前缀，
# 因此用「最后一个 wall<=odom_wall 的 LiDAR 回调」即可无损恢复 lidar 时基。
LOG_RE = re.compile(r"\[(\d+\.\d+)\] \[laserMapping\]: Get LiDAR, its header time: (\d+\.\d+)")


def _odom_raw(bagdir, topic):
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "builder", "/home/ywj/git/Super-LIVO/experiments/octvox_ablation/01_build_maps.py")
    B = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(B)
    od = B.read_bag(bagdir, [topic])[topic]
    tw = np.array([tt for tt, _ in od])
    p = np.array([[m.pose.pose.position.x, m.pose.pose.position.y,
                   m.pose.pose.position.z] for _, m in od])
    q = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                   m.pose.pose.orientation.z, m.pose.pose.orientation.w] for _, m in od])
    return tw, p, q


def load_frame_times(tag):
    """从 node.log 取 (wall_time, lidar_stamp) 序列。"""
    p = os.path.join(REF_ROOT, tag, "node.log")
    raw = open(p, "rb").read().decode("utf-8", "replace")
    rows = []
    for seg in re.split(r"[\r\n]+", raw):
        m = LOG_RE.search(seg)
        if m:
            rows.append((float(m.group(1)), float(m.group(2))))
    return np.array(rows)


def load_ref(tag, topic="/aft_mapped_to_init"):
    """返回 (lidar_stamp, xyz, quat, diag)。odom 的 wall 时间被换回 lidar 时基。"""
    tw, p, q = _odom_raw(os.path.join(REF_ROOT, tag, "bag"), topic)
    lf = load_frame_times(tag)
    # node.log 的 wall 前缀有 4 行比相邻行超前 ~2.85 s（控制台缓冲），直接
    # searchsorted 会在这 4 处跳 30 帧。丢掉这 4 行（仅损失 4 帧的时间标签）后
    # 数组单调，再取「最后一个 wall <= odom_wall 的 LiDAR 回调」。
    bad = np.where(np.diff(lf[:, 0]) < 0)[0] + 1
    keep = np.ones(len(lf), dtype=bool)
    keep[bad] = False
    lf = lf[keep]
    idx = np.searchsorted(lf[:, 0], tw, side="right") - 1
    ok = idx >= 0
    tw, p, q, idx = tw[ok], p[ok], q[ok], idx[ok]
    lag = tw - lf[idx, 0]                    # odom 发布相对该帧回调的延迟
    period = np.median(np.diff(lf[:, 1]))
    diag = dict(n_odom=int(len(tw)), n_lidar_cb=int(len(lf)),
                n_log_jumbled=int(len(bad)),
                n_unique_frame=int(len(np.unique(idx))),
                n_missing_frame=int(len(lf) - len(np.unique(idx))),
                lag_ms_p50=float(np.percentile(lag, 50) * 1e3),
                lag_ms_p90=float(np.percentile(lag, 90) * 1e3),
                lag_ms_max=float(lag.max() * 1e3),
                frame_period_ms=float(period * 1e3),
                lag_gt_period=int((lag > period).sum()))
    return lf[idx, 1], p, q, diag


def load_group(mode):
    a = np.loadtxt(os.path.join(CL_ROOT, f"rep_{mode}", "debug.csv"),
                   delimiter=",", skiprows=1)
    return a[:, 0], a[:, 14:17]


def kabsch(src, dst):
    """求 R,t 使 R@src+t 最接近 dst（scale 固定 1）。"""
    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ D @ U.T
    return R, cd - R @ cs


def pair(t_a, t_b, tol=TOL):
    """按时间戳最近邻配对（tol 内），返回 (ia, ib, dt_ms)。"""
    ia, ib, dts = [], [], []
    for i, t in enumerate(t_a):
        j = int(np.searchsorted(t_b, t))
        cand = [k for k in (j - 1, j, j + 1) if 0 <= k < len(t_b)]
        k = min(cand, key=lambda x: abs(t_b[x] - t))
        if abs(t_b[k] - t) <= tol:
            ia.append(i)
            ib.append(k)
            dts.append(abs(t_b[k] - t))
    return np.array(ia), np.array(ib), np.array(dts) * 1e3


def pstats(x):
    x = np.asarray(x, dtype=float)
    if len(x) == 0:
        return dict(n=0)
    return dict(n=int(len(x)), p50=float(np.percentile(x, 50)),
                p90=float(np.percentile(x, 90)), max=float(x.max()),
                rms=float(np.sqrt((x ** 2).mean())))


def jump_mask(t, p):
    """逐帧标记「该帧参与的前后两步都不超过 JUMP」。"""
    step = np.linalg.norm(np.diff(p, axis=0), axis=1)
    bad = np.zeros(len(p), dtype=bool)
    bad[1:] |= step > JUMP
    bad[:-1] |= step > JUMP
    return ~bad


def compare(name, t_ref, p_ref, t_grp, p_grp, mask_ref=None):
    """只在本分析窗内比较；给出全程 SE(3) 最优拟合后的逐帧偏差。
    mask_ref: 对 t_ref 的布尔掩码，限定只用「参考可信」的帧。"""
    ia, ib, dts = pair(t_grp, t_ref)
    tg, tr = p_grp[ia], p_ref[ib]
    R, t = kabsch(tg, tr)
    d = np.linalg.norm((R @ tg.T).T + t - tr, axis=1)
    ok = jump_mask(t_ref, p_ref)[ib] & jump_mask(t_grp, p_grp)[ia]
    if mask_ref is not None:
        ok &= mask_ref[ib]
    return dict(name=name, n_pair=int(len(tg)),
                pair_dt_ms_p50=float(np.percentile(dts, 50)),
                pair_dt_ms_max=float(dts.max()),
                fit_rot_deg=float(np.degrees(np.linalg.norm(Rot.from_matrix(R).as_rotvec()))),
                dev=pstats(d), dev_clean=pstats(d[ok]),
                jump_frames=int((~ok).sum()))


def main():
    res = {"window": list(WINDOW), "jump_thresh_m": JUMP, "items": [], "diag": {},
           "path_len_m": {}}

    def win(t, p):
        k = (t >= WINDOW[0]) & (t <= WINDOW[1])
        return t[k], p[k]

    refs = {}
    for tag in ("ref1", "ref2"):
        t, p, q, d = load_ref(tag)
        tw_, pw_ = win(t, p)
        refs[tag] = (tw_, pw_)
        d["in_window_frames"] = int(len(tw_))
        d["in_window_jump_frames"] = int((~jump_mask(tw_, pw_)).sum())
        d["in_window_path_len_m"] = float(np.linalg.norm(np.diff(pw_, axis=0), axis=1).sum())
        res["diag"][tag] = d

    t1, p1 = refs["ref1"]
    res["items"].append(compare("ref2_vs_ref1", t1, p1, *refs["ref2"]))
    for mode in ("A", "B", "C"):
        tg, pg = win(*load_group(mode))
        res["path_len_m"][mode] = float(np.linalg.norm(np.diff(pg, axis=0), axis=1).sum())
        res["items"].append(compare(f"{mode}_vs_ref1", t1, p1, tg, pg))
    res["path_len_m"]["ref1"] = res["diag"]["ref1"]["in_window_path_len_m"]
    res["path_len_m"]["ref2"] = res["diag"]["ref2"]["in_window_path_len_m"]

    # 「参考可信」子集：两轮参考互差最小的那一半帧。两轮一致说明该处 FAST-LIVO2
    # 自身可重复，此时它才有资格当参考；用它比较 A/B/C 可避开两轮不一致的瞬态帧。
    t2, p2 = refs["ref2"]
    ia, ib, _ = pair(t2, t1)
    R12, t12 = kabsch(p2[ia], p1[ib])
    d12 = np.linalg.norm((R12 @ p2[ia].T).T + t12 - p1[ib], axis=1)
    thr = float(np.median(d12))
    trusted = np.zeros(len(t1), dtype=bool)
    trusted[ib] = d12 <= thr
    res["trusted"] = dict(rule="两轮参考互差 <= 中位数", thresh_m=thr,
                          n_frames=int(trusted.sum()), of=int(len(t1)))
    res["trusted_items"] = [compare(f"{m}_vs_ref1", t1, p1, *win(*load_group(m)), mask_ref=trusted)
                            for m in ("A", "B", "C")]

    with open(os.path.join(OUT, "external_comparison.json"), "w") as f:
        json.dump(res, f, indent=1)

    hdr = f"{'pair':<14}{'n':>4}{'jump':>5}{'dt50':>6}{'rot':>7}" \
          f"{'all P50':>9}{'P90':>8}{'RMS':>8}   {'clean P50':>10}{'P90':>8}{'RMS':>8}{'n':>5}"
    print(hdr)
    for it in res["items"]:
        a, c = it["dev"], it["dev_clean"]
        print(f"{it['name']:<14}{it['n_pair']:>4}{it['jump_frames']:>5}"
              f"{it['pair_dt_ms_p50']:>6.2f}{it['fit_rot_deg']:>7.2f}"
              f"{a['p50']*1e3:>8.1f}m{a['p90']*1e3:>7.1f}{a['rms']*1e3:>7.1f}   "
              f"{c['p50']*1e3:>9.1f}m{c['p90']*1e3:>7.1f}{c['rms']*1e3:>7.1f}{c['n']:>5}")
    print("path_len in window (m):",
          {k: round(v, 3) for k, v in res["path_len_m"].items()})
    for tag in ("ref1", "ref2"):
        print(tag, res["diag"][tag])

    print(f"\n参考可信子集（{res['trusted']['rule']} = {res['trusted']['thresh_m']*1e3:.0f} mm）："
          f"{res['trusted']['n_frames']}/{res['trusted']['of']} 帧")
    print(f"{'pair':<14}{'n':>5}{'P50':>9}{'P90':>8}{'RMS':>8}{'max':>8}")
    for it in res["trusted_items"]:
        c = it["dev_clean"]
        if c.get("n", 0) == 0:
            print(f"{it['name']:<14} 无样本")
            continue
        print(f"{it['name']:<14}{c['n']:>5}{c['p50']*1e3:>8.1f}m{c['p90']*1e3:>7.1f}"
              f"{c['rms']*1e3:>7.1f}{c['max']*1e3:>7.1f}")


if __name__ == "__main__":
    main()
