#!/usr/bin/env python3
# 官方 FAST-LIVO2「全流程(LIVO, img_en=1)」vs「去除图像(img_en=0)」两种条件的 odom 差异分析。
#
#   python3 tools/fl2_odo_diff.py
#
# 所有轨迹量先在统一 10 Hz 时间网格上重采样后再算，避免 img_en=0 那一路
# 不规则 dt（0.5~157 ms）污染二阶量。两条轨迹之间的常数时基偏差用
# 网格重采样后的互相关搜索求出（±0.3 s，步长 5 ms）。
import importlib.util
import json
import os
import re
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

SLAM = "/home/ywj/slam_data"
LIVO_TXT = f"{SLAM}/pcd/retail_street_livo_official/retail_street_livo_poses.txt"
LIO_TXT = f"{SLAM}/retail_abl/fl2_lio_aligned.txt"
LIO_BAG = f"{SLAM}/retail_abl/fl2_lio"
LIO_LOG = "/tmp/fl2_lio.log"
REF_ROOT = f"{SLAM}/fastlivo2_ref_20261110"
JUMP = 0.3          # 单帧位移阈值(m)
GRID = 0.1          # 统一网格 (Hz^-1)，与 LiDAR 帧周期一致
LOG_RE = re.compile(r"\[(\d+\.\d+)\] \[laserMapping\]: Get LiDAR, its header time: (\d+\.\d+)")


def tum(path):
    a = np.loadtxt(path)
    k = np.argsort(a[:, 0])
    return a[k, 0], a[k, 1:4], a[k, 4:8]


def dedupe(t, p, q):
    """按 lidar 时基排序；同一帧有多条 odom 时保留最后一条（后验更新）。"""
    k = np.argsort(t, kind="stable")
    t, p, q = t[k], p[k], q[k]
    # 同帧多条时取该组的最后一条
    last_of = {}
    for i, tt in enumerate(t):
        last_of[tt] = i
    sel = np.zeros(len(t), bool)
    sel[list(last_of.values())] = True
    return t[sel], p[sel], q[sel]


def ref_poses(tag):
    """ref1/ref2：odom header 是 wall clock，用 node.log 换回 lidar 时基。"""
    tw, p, q = odom_raw(os.path.join(REF_ROOT, tag, "bag"))
    lf = lidar_wall_pairs(os.path.join(REF_ROOT, tag, "node.log"), tw.min() - 5, tw.max() + 5)
    idx = np.searchsorted(lf[:, 0], tw, side="right") - 1
    ok = idx >= 0
    return dedupe(lf[idx[ok], 1], p[ok], q[ok])


def resample(t, p, q, grid):
    """线性插位置 + 四元数球面插值，采到统一网格 grid 上。"""
    k = Rot.from_quat(q)
    out_p = np.empty((len(grid), 3))
    for a in range(3):
        out_p[:, a] = np.interp(grid, t, p[:, a])
    # scipy Slerp 要求 grid 在 [t0, tn] 内
    from scipy.spatial.transform import Slerp
    sl = Slerp(t, k)
    out_q = sl(np.clip(grid, t[0], t[-1])).as_quat()
    return out_p, out_q


def grid_of(t_a, t_b, shift, margin=0.5):
    lo = max(t_a[0], t_b[0] + shift) + margin
    hi = min(t_a[-1], t_b[-1] + shift) - margin
    return np.arange(lo, hi, GRID)


def _build(name):
    spec = importlib.util.spec_from_file_location(
        name, "/home/ywj/git/Super-LIVO/experiments/octvox_ablation/01_build_maps.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def odom_raw(bagdir, topic="/aft_mapped_to_init"):
    B = _build("builder")
    od = B.read_bag(bagdir, [topic])[topic]
    tw = np.array([t for t, _ in od])
    p = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
                  for _, m in od])
    q = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                   m.pose.pose.orientation.z, m.pose.pose.orientation.w] for _, m in od])
    return tw, p, q


def lidar_wall_pairs(logpath, wlo, whi):
    """node.log 里属于 [wlo,whi] 这段运行的 (wall, lidar_stamp) 序列。"""
    raw = open(logpath, "rb").read().decode("utf-8", "replace")
    rows = [(float(m.group(1)), float(m.group(2)))
            for m in (LOG_RE.search(s) for s in re.split(r"[\r\n]+", raw)) if m]
    rows = [(w, s) for w, s in rows if wlo <= w <= whi]
    arr = np.array(rows)
    if len(arr) == 0:
        return arr
    bad = np.where(np.diff(arr[:, 0]) < 0)[0] + 1
    keep = np.ones(len(arr), bool)
    keep[bad] = False
    return arr[keep]


def lio_poses():
    """img_en=0 那一路：把 odom 的 wall clock 换回 lidar 时基（与 LIVO txt 同域）。"""
    tw, p, q = odom_raw(LIO_BAG)
    lf = lidar_wall_pairs(LIO_LOG, tw.min() - 5, tw.max() + 5)
    if len(lf) < 0.8 * len(tw):
        raise RuntimeError(f"node.log 只匹配到 {len(lf)} 条 LiDAR 回调 / odom {len(tw)} 条")
    idx = np.searchsorted(lf[:, 0], tw, side="right") - 1
    ok = idx >= 0
    lag = tw - lf[idx, 0]
    to, po, qo = dedupe(lf[idx[ok], 1], p[ok], q[ok])
    print(f"[时基恢复] odom {len(tw)} 条 → 唯一 lidar 帧 {len(to)} 个"
          f"（无位姿 {len(lf)-len(to)}），发布延迟 中位 {np.median(lag)*1e3:.1f} ms "
          f"P90 {np.percentile(lag,90)*1e3:.1f} max {lag.max()*1e3:.1f} ms")
    return to, po, qo


def sg_resid(p, win=5):
    half = win // 2
    vv = np.arange(-half, half + 1, dtype=float)
    A = np.vstack([vv ** k for k in range(3)]).T
    C = np.linalg.pinv(A)[0]
    r = np.stack([p[half:len(p) - half, a] - np.convolve(p[:, a], C[::-1], "valid")
                  for a in range(3)], axis=1)
    return np.linalg.norm(r, axis=1)


def selfstats(name, t, p, q, note=""):
    dt = np.diff(t)
    step = np.linalg.norm(np.diff(p, axis=0), axis=1)
    Rs = Rot.from_quat(q).as_matrix()
    dang = np.degrees(np.linalg.norm(
        Rot.from_matrix(np.einsum("nij,njk->nik", Rs[:-1].transpose(0, 2, 1), Rs[1:])).as_rotvec(),
        axis=1))
    jit = sg_resid(p)
    span = t[-1] - t[0]
    print(f"\n--- {name} --- {note}")
    print(f"帧数 {len(t)}  时长 {span:.1f}s  dt {np.median(dt)*1e3:.1f}ms "
          f"(min {dt.min()*1e3:.1f} / max {dt.max()*1e3:.1f})  原始 dt 不规则 → 已重采样到 10Hz 网格")
    print(f"路径长 {step.sum():.2f} m   平均速度 {step.sum()/span:.2f} m/s")
    print(f"帧间位移 中位 {np.median(step)*1e3:.1f} P90 {np.percentile(step,90)*1e3:.1f} "
          f"P99 {np.percentile(step,99)*1e3:.1f} max {step.max()*1e3:.1f} mm")
    print(f"瞬态跳变(>{JUMP*1e3:.0f}mm) {int((step>JUMP).sum())} 帧 "
          f"({100*(step>JUMP).mean():.1f}%)，累计虚长 {step[step>JUMP].sum():.1f} m")
    print(f"帧间转角 中位 {np.median(dang):.2f} P90 {np.percentile(dang,90):.2f} "
          f"P99 {np.percentile(dang,99):.2f} max {dang.max():.2f} °")
    print(f"高频抖动(5帧SG残差) 中位 {np.median(jit)*1e3:.1f} P90 {np.percentile(jit,90)*1e3:.1f} "
          f"RMS {np.sqrt((jit**2).mean())*1e3:.1f} mm")
    print(f"自身系回环 位移 {np.linalg.norm(p[-1]-p[0])*1e3:.1f} mm  "
          f"姿态 {np.degrees(np.linalg.norm(Rot.from_matrix(Rs[0].T@Rs[-1]).as_rotvec())):.2f} °")
    return dict(n=len(t), path=float(step.sum()),
                step_p99=float(np.percentile(step, 99)),
                jumps=int((step > JUMP).sum()),
                jit_med=float(np.median(jit)), jit_rms=float(np.sqrt((jit ** 2).mean())),
                loop_dp=float(np.linalg.norm(p[-1] - p[0])))


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    C = (dst - mu_d).T @ (src - mu_s) / len(src)
    U, sv, Vt = np.linalg.svd(C)
    R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    return R, mu_d - R @ mu_s


def ang_between(Ra, Rb):
    return np.degrees(np.linalg.norm(
        Rot.from_matrix(np.einsum("nij,njk->nik", Ra.transpose(0, 2, 1), Rb)).as_rotvec(), axis=1))


def compare(name, ta, pa, qa, tb, pb, qb):
    """b 相对 a。搜索常数时基偏移使 1s 互位移残差最小，然后同网格重采样比较。"""
    best = None
    scan = {}
    for sh in np.arange(-0.30, 0.301, 0.005):
        g = grid_of(ta, tb, sh)
        if len(g) < 200:
            continue
        pa_g, qa_g = resample(ta, pa, qa, g)
        pb_g, qb_g = resample(tb + sh, pb, qb, g)
        R, tvec = umeyama(pb_g, pa_g)
        d = np.linalg.norm(pa_g - ((R @ pb_g.T).T + tvec), axis=1)
        score = np.median(d)
        ms = int(round(sh * 1e3))
        if ms % 50 == 0:
            scan[ms] = score
        if best is None or score < best[0]:
            best = (score, sh, g, pa_g, qa_g, pb_g, qb_g, R, tvec, d)
    score, sh, g, pa_g, qa_g, pb_g, qb_g, R, tvec, d = best
    pb_al = (R @ pb_g.T).T + tvec
    ang = ang_between(Rot.from_quat(qa_g).as_matrix(),
                      (R @ Rot.from_quat(qb_g).as_matrix()))
    print(f"\n--- {name} ---")
    print(f"最优时基偏移 {sh*1e3:+.0f} ms（中位距离残差 {score*1e3:.0f} mm），共同网格 {len(g)} 点")
    print("           时基扫描中位残差 mm: " +
          " ".join(f"{k:+d}:{v*1e3:.0f}" for k, v in sorted(scan.items())))
    print(f"SE3对齐后位置差 均值 {d.mean()*1e3:.0f} 中位 {np.median(d)*1e3:.0f} "
          f"P90 {np.percentile(d,90)*1e3:.0f} max {d.max()*1e3:.0f} mm")
    print(f"           旋转差 均值 {ang.mean():.2f} 中位 {np.median(ang):.2f} "
          f"P90 {np.percentile(ang,90):.2f} max {ang.max():.2f} °")
    dp = pa_g - pb_al
    print(f"           分轴 dp x {dp[:,0].mean()*1e3:+.0f}(std {dp[:,0].std()*1e3:.0f}) "
          f"y {dp[:,1].mean()*1e3:+.0f}(std {dp[:,1].std()*1e3:.0f}) "
          f"z {dp[:,2].mean()*1e3:+.0f}(std {dp[:,2].std()*1e3:.0f}) mm")
    kern = np.ones(10) / 10
    lp = np.stack([np.convolve(dp[:, a], kern, "same") for a in range(3)], axis=1)
    hp = dp - lp
    print(f"           差值分解 低频(1s滑动均值) RMS {np.sqrt((lp**2).sum(1).mean())*1e3:.0f} mm | "
          f"高频(去低频) RMS {np.sqrt((hp**2).sum(1).mean())*1e3:.0f} mm")
    tt = g - g[0]
    seg = [f"{k*10:.0f}s:{np.median(d[(tt>=k*10)&(tt<(k+1)*10)])*1e3:.0f}"
           for k in range(int(tt[-1] // 10) + 1)]
    print("           分 10s 段位置差中位 mm:", " ".join(seg))
    res = {"shift_ms": float(sh * 1e3), "ate_mean_mm": float(d.mean() * 1e3),
           "ate_p50_mm": float(np.median(d) * 1e3), "ate_p90_mm": float(np.percentile(d, 90) * 1e3),
           "ang_mean_deg": float(ang.mean()), "lp_rms_mm": float(np.sqrt((lp ** 2).sum(1).mean()) * 1e3),
           "hp_rms_mm": float(np.sqrt((hp ** 2).sum(1).mean()) * 1e3), "rpe": {}}
    for s, lab in ((10, "1s"), (50, "5s"), (100, "10s")):
        if len(g) <= s:
            continue
        rpe = np.abs(np.linalg.norm(pa_g[s:] - pa_g[:-s], axis=1) -
                     np.linalg.norm(pb_al[s:] - pb_al[:-s], axis=1))
        print(f"           RPE({lab}) 位移差 中位 {np.median(rpe)*1e3:.0f} "
              f"P90 {np.percentile(rpe,90)*1e3:.0f} mm")
        res["rpe"][lab] = [float(np.median(rpe) * 1e3), float(np.percentile(rpe, 90) * 1e3)]
    return res


def main():
    out = {}
    tl, pl, ql = tum(LIVO_TXT)
    try:
        tio, pio, qio = lio_poses()
        lio_note = "11-08 产物，odom 已换回 lidar 时基"
    except Exception as e:  # noqa
        tio, pio, qio = tum(LIO_TXT)
        lio_note = f"11-08 产物，仅常数时基平移（node.log 不可用：{e}）"
    print("=" * 78)
    print("官方 FAST-LIVO2 —— 两种条件（同 bag: Retail_Street，只差 img_en）")
    print("=" * 78)
    # 自身一致性量：统一网格重采样后计算
    g = grid_of(tl, tio, 0.0)
    for nm, (t_, p_, q_), note in ((
            "LIVO img_en=1", (tl, pl, ql), "11-06 产物，lidar 时基"),
            ("img_en=0      ", (tio, pio, qio), lio_note)):
        pg, qg = resample(t_, p_, q_, g)
        out[nm] = selfstats(nm, g, pg, qg, note)
    out["pair_livo_lio"] = compare("LIVO(img_en=1) 减 img_en=0 【lidar 时基，正确口径】",
                                   tl, pl, ql, tio, pio, qio)
    # 同条件跨轮噪声底（11-06 v1 vs 11-07 v2，均为 img_en=1）
    tv2, pv2, qv2 = tum(f"{SLAM}/pcd/retail_street_livo_official/retail_street_livo_poses_v2.txt")
    out["livo_v1_vs_v2"] = compare("LIVO(img_en=1) v1 减 v2（同条件跨轮 = 噪声底）",
                                   tl, pl, ql, tv2, pv2, qv2)
    # 对照：复刻 HANDOVER 当时的口径（odom header 墙钟 + 仅常数平移，不做发布延迟改正）
    twall, pwall, qwall = tum(LIO_TXT)
    out["pair_livo_lio_wallclock"] = compare(
        "LIVO(img_en=1) 减 img_en=0 【对照：未做时基恢复，= HANDOVER 当时口径】",
        tl, pl, ql, twall, pwall, qwall)

    try:
        t1, p1, q1 = ref_poses("ref1")
        t2, p2, q2 = ref_poses("ref2")
        print("\n" + "=" * 78)
        print("同条件重复性（11-10 两次官方 LIVO 重跑）= 噪声底")
        print("=" * 78)
        g2 = grid_of(t1, t2, 0.0)
        for nm, (t_, p_, q_) in (("ref1", (t1, p1, q1)), ("ref2", (t2, p2, q2))):
            pg, qg = resample(t_, p_, q_, g2)
            out[nm] = selfstats(nm + " (LIVO 重跑)", g2, pg, qg, "11-10 产物")
        out["ref1_vs_ref2"] = compare("ref1 减 ref2（同条件跨轮 = 噪声底）", t1, p1, q1, t2, p2, q2)
        out["livo06_vs_ref1"] = compare("LIVO(11-06) 减 ref1(11-10)（同条件跨日）",
                                        tl, pl, ql, t1, p1, q1)
    except Exception as e:  # noqa
        print(f"\n[跳过] ref1/ref2 读取失败: {type(e).__name__}: {e}")

    dst = os.environ.get("FL2_OUT", SLAM)
    with open(os.path.join(dst, "fl2_odo_diff.json"), "w") as f:
        json.dump(out, f, indent=1, ensure_ascii=False)
    print(f"\n已写出 {dst}/fl2_odo_diff.json")


if __name__ == "__main__":
    sys.exit(main())
