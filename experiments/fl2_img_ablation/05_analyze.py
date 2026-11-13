#!/usr/bin/env python3
"""FAST-LIVO2 有/无图像成对消融的结果分析。

A. NTU eee_03（有真值）：每轮 ATE + 按条件分组 + 逐轮配对差；
   并用新跑的 6 轮去"指纹比对"两份来历不明的旧轨迹
   （Log/result/eee_03.txt / eee_03_lio.txt），反推它们各自的 img_en。
B. Retail_Street（无真值）：每轮自身一致性 + 跨条件轨迹差 vs 同条件轮次差（噪声底）。

用法：python3 05_analyze.py
输出：打印 + /home/ywj/git/Super-LIVO/fl2_img_ablation_result.json
"""
import glob
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

OUT_AB = "/home/ywj/slam_data/fl2_img_abl_20261110"
WS = "/home/ywj/git/Super-LIVO"
GT = "/home/ywj/slam_data/ntu_viral/gt/eee_03/ground_truth.csv"
FLLOG = f"{WS}/third_party/FAST-LIVO2-ROS2/Log/result"
JUMP = 0.3
FX = 646.78

sys.path.insert(0, f"{WS}/tools")
import odom_vs_gt as og  # noqa: E402  复用官方 NTU 评测协议实现


def tum(path):
    a = np.loadtxt(path)
    if a.ndim == 1:
        a = a[None, :]
    k = np.argsort(a[:, 0])
    return a[k, 0], a[k, 1:4], a[k, 4:8]


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    C = (dst - mu_d).T @ (src - mu_s) / len(src)
    U, _, Vt = np.linalg.svd(C)
    R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    return R, mu_d - R @ mu_s


def pair_seq(ta, tb, jitter=0.020):
    """两条同 bag 的位姿序列做单调贪心对齐。
    img_en=1 那一路的 last_lio_update_time 带若干 ms 抖动（多走一遍图像同步），
    且两种条件丢的帧不同，因此既不能按索引配、也不能用固定容差最近邻配。
    jitter 取 20 ms：NTU 帧周期 83 ms、Retail 帧周期 99.85 ms，均远大于该窗口，
    不会把相邻帧错配。"""
    ia, ib = [], []
    i = j = 0
    while i < len(ta) and j < len(tb):
        dt = ta[i] - tb[j]
        if abs(dt) <= jitter:
            ia.append(i); ib.append(j); i += 1; j += 1
        elif dt < 0:
            i += 1
        else:
            j += 1
    ia = np.array(ia); ib = np.array(ib)
    return ia, ib, np.abs(ta[ia] - tb[ib])


def traj_diff(pa, qa, pb, qb, ia, ib):
    R, t = umeyama(pb[ib], pa[ia])
    pb_al = (R @ pb[ib].T).T + t
    d = np.linalg.norm(pa[ia] - pb_al, axis=1)
    Ra = Rot.from_quat(qa[ia]).as_matrix()
    Rb = R @ Rot.from_quat(qb[ib]).as_matrix()
    ang = np.degrees(np.linalg.norm(
        Rot.from_matrix(np.einsum("nij,njk->nik", Ra.transpose(0, 2, 1), Rb)).as_rotvec(), axis=1))
    dp = pa[ia] - pb_al
    kern = np.ones(10) / 10
    lp = np.stack([np.convolve(dp[:, a], kern, "same") for a in range(3)], axis=1)
    return dict(n=int(len(ia)), dt_p50_ms=None,
                ident_frac=float((d < 1e-4).mean()),
                pos_mean_mm=float(d.mean() * 1e3), pos_p50_mm=float(np.median(d) * 1e3),
                pos_p90_mm=float(np.percentile(d, 90) * 1e3), pos_max_mm=float(d.max() * 1e3),
                ang_mean_deg=float(ang.mean()), ang_p50_deg=float(np.median(ang)),
                lp_rms_mm=float(np.sqrt((lp ** 2).sum(1).mean()) * 1e3),
                hp_rms_mm=float(np.sqrt(((dp - lp) ** 2).sum(1).mean()) * 1e3))


def selfstats(t, p, q):
    step = np.linalg.norm(np.diff(p, axis=0), axis=1)
    Rs = Rot.from_quat(q).as_matrix()
    jit_v = np.arange(-2, 3, dtype=float)
    C = np.linalg.pinv(np.vstack([jit_v ** k for k in range(3)]).T)[0]
    jit = np.linalg.norm(np.stack([p[2:-2, a] - np.convolve(p[:, a], C[::-1], "valid")
                                   for a in range(3)], axis=1), axis=1)
    return dict(n=len(t), span_s=float(t[-1] - t[0]), path_m=float(step.sum()),
                step_med_mm=float(np.median(step) * 1e3),
                step_p99_mm=float(np.percentile(step, 99) * 1e3),
                jumps=int((step > JUMP).sum()), jit_med_mm=float(np.median(jit) * 1e3),
                loop_dp_mm=float(np.linalg.norm(p[-1] - p[0]) * 1e3),
                loop_ang_deg=float(np.degrees(np.linalg.norm(
                    Rot.from_matrix(Rs[0].T @ Rs[-1]).as_rotvec()))))


def fmt(d, keys):
    return " | ".join(f"{k}={d[k]:.2f}" if isinstance(d[k], float) else f"{k}={d[k]}"
                      for k in keys)


def ate_at(est, gt, shift=0.0):
    """把估计时间整体平移 shift 秒后按官方协议算 ATE。"""
    e = est.copy()
    e[:, 0] += shift
    t = e[:, 0]
    p = e[:, 1:4].copy()
    for i in range(len(p)):
        p[i] += og.quat_to_rot(e[i, 4:8]) @ og.T_B_PRISM
    pg, ok = og.interp_gt_at(gt, t)
    if ok.sum() < 20:
        return float("nan"), 0
    pe, pgg = p[ok], pg[ok]
    R, tr = og.umeyama_se3(pe, pgg)
    err = np.linalg.norm((R @ pe.T).T + tr - pgg, axis=1)
    return float(np.sqrt(np.mean(err ** 2))), int(ok.sum())


def ate_sweep(est, gt, lo=-0.03, hi=0.03, step=0.002):
    """±30 ms 时基敏感性：img_en=1 那一路的 last_lio_update_time 带约 9 ms 系统偏移，
    必须证明结论不是时基偏差造成的，才能把 ATE 差归给视觉。"""
    a0, _ = ate_at(est, gt, 0.0)
    best = (a0, 0.0)
    for sh in np.arange(lo, hi + 1e-9, step):
        a, _ = ate_at(est, gt, sh)
        if a == a and a < best[0]:
            best = (a, sh)
    return a0, best[0], best[1]


def main():
    res = {"ntu": {}, "retail": {}}
    print("=" * 90)
    print("A. NTU eee_03 —— 有真值，判决性")
    print("=" * 90)
    gt = og.load_gt(GT)
    runs = sorted(glob.glob(f"{OUT_AB}/ntu/m*_pose.txt"))
    WARMUP = 15.0
    ate = {}
    for f in runs:
        tag = os.path.basename(f).replace("_pose.txt", "")
        est = og.load_odom(f, "/none")
        if est is None:
            print(f"{tag}: 轨迹过短，跳过"); continue
        r = og.evaluate(f"官方 img_en={tag[1]} rep={tag[3]}  全程", est, gt)
        rt = og.evaluate(f"官方 img_en={tag[1]} rep={tag[3]}  去前{WARMUP:.0f}s",
                         est[est[:, 0] > est[0, 0] + WARMUP], gt)
        ate[tag] = dict(ate_m=float(r["ate"]), ate_steady=float(rt["ate"]),
                        med=float(np.median(r["err"])), p95=float(np.percentile(r["err"], 95)),
                        max=float(r["err"].max()), n=len(r["err"]))
        a0, abest, ashift = ate_sweep(est, gt)
        ate[tag].update(ate_sweep_best=abest, ate_sweep_shift_ms=float(ashift * 1e3))
        print(f"    → 时基敏感性：原 {a0:.3f} m，±30 ms 内最优 {abest:.3f} m @ {ashift*1e3:+.0f} ms")
    print(f"\n  汇总（全程 ATE / 稳态 ATE / 各自时基最优 ATE，单位 m）")
    for m in ("1", "0"):
        full = [a["ate_m"] for k, a in ate.items() if k[1] == m]
        steady = [a["ate_steady"] for k, a in ate.items() if k[1] == m]
        sw = [a["ate_sweep_best"] for k, a in ate.items() if k[1] == m]
        if full:
            print(f"    img_en={m}: 全程 {np.mean(full):.3f}±{np.std(full)*1e3:.0f}mm  "
                  f"稳态 {np.mean(steady):.3f}±{np.std(steady)*1e3:.0f}mm  "
                  f"时基最优 {np.mean(sw):.3f}±{np.std(sw)*1e3:.0f}mm  "
                  f"（逐轮全程 {['%.3f' % v for v in full]}）")
        res["ntu"][f"img_en{m}"] = dict(ate_list=full, steady_list=steady, sweep_list=sw,
                                        mean=float(np.mean(full)), std=float(np.std(full)),
                                        steady_mean=float(np.mean(steady)),
                                        steady_std=float(np.std(steady)),
                                        sweep_mean=float(np.mean(sw)),
                                        sweep_std=float(np.std(sw)))
    paired = []
    paired_steady = []
    paired_sweep = []
    for r in sorted({k[3] for k in ate}):
        k1, k0 = f"m1r{r}", f"m0r{r}"
        if k1 in ate and k0 in ate:
            paired.append(ate[k0]["ate_m"] - ate[k1]["ate_m"])
            paired_steady.append(ate[k0]["ate_steady"] - ate[k1]["ate_steady"])
            paired_sweep.append(ate[k0]["ate_sweep_best"] - ate[k1]["ate_sweep_best"])
    if paired:
        res["ntu"]["paired_gain_m"] = paired
        res["ntu"]["paired_gain_steady_m"] = paired_steady
        res["ntu"]["paired_gain_sweep_m"] = paired_sweep
        print(f"  逐轮配对（无图像 − 有图像，正 = 有图像更准）："
              f"全程 " + " ".join(f"{x*1e3:+.0f}" for x in paired) +
              f" mm（均值 {np.mean(paired)*1e3:+.0f}） | 稳态 " +
              " ".join(f"{x*1e3:+.0f}" for x in paired_steady) +
              f"（均值 {np.mean(paired_steady)*1e3:+.0f}） | 各自时基最优 " +
              " ".join(f"{x*1e3:+.0f}" for x in paired_sweep) +
              f"（均值 {np.mean(paired_sweep)*1e3:+.0f}）")

    print("\n--- 旧轨迹指纹比对：eee_03.txt / eee_03_lio.txt 到底是哪种模式 ---")
    ident = {}
    for old in ("eee_03.txt", "eee_03_lio.txt"):
        pth = f"{FLLOG}/{old}"
        if not os.path.exists(pth):
            print(f"  {old} 不存在"); continue
        to, po, qo = tum(pth)
        row = {}
        for f in runs:
            tag = os.path.basename(f).replace("_pose.txt", "")
            tn, pn, qn = tum(f)
            ia, ib, _ = pair_seq(to, tn)
            if len(ia) < 200:
                continue
            row[tag] = traj_diff(po, qo, pn, qn, ia, ib)["pos_p50_mm"]
        ident[old] = row
        best = min(row, key=row.get) if row else "NA"
        print(f"  {old}: " + " ".join(f"{k}={v:.0f}mm" for k, v in sorted(row.items())) +
              f"  → 最像 {best}")
    res["ntu"]["legacy_fingerprint_p50_mm"] = ident

    print("\n--- 跨条件轨迹差 vs 同条件轮次差（NTU，lidar 时基逐帧配对）---")
    ntud = {}
    poses = {os.path.basename(f).replace("_pose.txt", ""): f
             for f in glob.glob(f"{OUT_AB}/ntu/m*_pose.txt")}
    combos = [("m1r1", "m0r1"), ("m1r2", "m0r2"), ("m1r3", "m0r3"),
              ("m1r1", "m1r2"), ("m1r2", "m1r3"), ("m1r1", "m1r3"),
              ("m0r1", "m0r2"), ("m0r2", "m0r3"), ("m0r1", "m0r3")]
    for a, b in combos:
        if a not in poses or b not in poses:
            continue
        ta, pa, qa = tum(poses[a]); tb, pb, qb = tum(poses[b])
        ia, ib, dt = pair_seq(ta, tb)
        d = traj_diff(pa, qa, pb, qb, ia, ib)
        d["dt_p50_ms"] = float(np.median(dt) * 1e3)
        kind = "跨条件" if a[1] != b[1] else "同条件"
        print(f"  [{kind}] {a} vs {b}: 配对 {d['n']} dt {d['dt_p50_ms']:.2f}ms "
              f"逐帧相同 {d['ident_frac']*100:.0f}% | "
              f"位置 中位 {d['pos_p50_mm']:.0f} P90 {d['pos_p90_mm']:.0f} mm | "
              f"旋转 中位 {d['ang_p50_deg']:.2f}° | 低频 {d['lp_rms_mm']:.0f} 高频 {d['hp_rms_mm']:.0f} mm")
        ntud[f"{a}_vs_{b}"] = d
    res["ntu"]["traj_diff"] = ntud

    print("\n" + "=" * 90)
    print("B. Retail_Street —— 无真值， odom + PCD 成对资产 + 轨迹一致性")
    print("=" * 90)
    rp = {}
    for f in sorted(glob.glob(f"{OUT_AB}/retail/m*r*/pose.txt")):
        tag = f.split("/")[-2]
        t, p, q = tum(f)
        s = selfstats(t, p, q)
        rp[tag] = s
        print(f"  {tag}: " + fmt(s, ["n", "path_m", "step_med_mm", "step_p99_mm", "jumps",
                                     "jit_med_mm", "loop_dp_mm", "loop_ang_deg"]))
    res["retail"]["selfstats"] = rp
    rd = {}
    for a, b in combos:
        fa, fb = f"{OUT_AB}/retail/{a}/pose.txt", f"{OUT_AB}/retail/{b}/pose.txt"
        if not (os.path.exists(fa) and os.path.exists(fb)):
            continue
        ta, pa, qa = tum(fa); tb, pb, qb = tum(fb)
        ia, ib, dt = pair_seq(ta, tb)
        d = traj_diff(pa, qa, pb, qb, ia, ib)
        d["dt_p50_ms"] = float(np.median(dt) * 1e3)
        kind = "跨条件" if a[1] != b[1] else "同条件"
        print(f"  [{kind}] {a} vs {b}: 配对 {d['n']} dt {d['dt_p50_ms']:.2f}ms | "
              f"位置 中位 {d['pos_p50_mm']:.0f} 均值 {d['pos_mean_mm']:.0f} "
              f"P90 {d['pos_p90_mm']:.0f} mm | 旋转 中位 {d['ang_p50_deg']:.2f}° | "
              f"低频 {d['lp_rms_mm']:.0f} 高频 {d['hp_rms_mm']:.0f} mm")
        rd[f"{a}_vs_{b}"] = d
    res["retail"]["traj_diff"] = rd
    cross = [v["pos_p50_mm"] for k, v in rd.items() if k.split("_vs_")[0][1] != k.split("_vs_")[1][1]]
    same = [v["pos_p50_mm"] for k, v in rd.items() if k.split("_vs_")[0][1] == k.split("_vs_")[1][1]]
    if cross and same:
        res["retail"]["summary"] = dict(cross_med_mm=float(np.median(cross)),
                                        same_med_mm=float(np.median(same)),
                                        ratio=float(np.median(cross) / np.median(same)))
        print(f"\n  Retail 汇总：跨条件（有/无图像）互差中位 {np.median(cross):.0f} mm，"
              f"同条件轮次互差中位 {np.median(same):.0f} mm，比值 {np.median(cross)/np.median(same):.1f}×")
        print(f"  折算重投影：{np.median(cross):.0f} mm @10 m = "
              f"{np.median(cross)/1e3/10*FX:.2f} px")

    dst = f"{WS}/fl2_img_ablation_result.json"
    with open(dst, "w") as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print(f"\n已写出 {dst}")


if __name__ == "__main__":
    main()
