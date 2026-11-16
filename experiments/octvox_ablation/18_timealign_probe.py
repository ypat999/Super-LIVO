#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P0-A 时间戳与位姿插值核验 —— 判决"有没有把真实运动错当抖动"，实测 Δt_camera。

为什么必须做：v1 写死 `LIO 时间戳 − 50 ms`，评审指出这只在特定相位关系下成立；
20 °/s 转向时 50 ms = 1 °，比 0.2 ° 目标大 5 倍。本脚本用数据钉死。

  A. 戳结构：image 的 header.stamp 是否逐帧等于 lidar 的 header.stamp；
     odom 戳对齐 LiDAR 扫描起点 / 中点 / 终点哪一个。
  B. LIO 连续轨迹（slerp）vs IMU 姿态积分，扫偏置 a 求
        median ∠( [A(t2)ᵀA(t1)]^T · [I(t2)ᵀI(t1)] )   最小，a∈[−150,150] ms
     —— 核验"位姿插值 + 戳语义"是否自洽。
  C. 图像相对旋转 vs IMU 姿态积分，扫 Δt_camera 求
        median ∠( R_vis(k→k+1)ᵀ · R_cl[I(t2)ᵀI(t1)]R_clᵗ )  最小
     IMU 200 Hz 在 ±100 ms 尺度上角速度真实变化，梯度足；
     v1 只拿 10 Hz LIO 位姿做互相关，时间分辨率与激励都不够。
     （R_cl 的侧别为**实测 b→c**，见 _p0_visual 模块头；旧版本此处写成 R_clᵀ·[·]·R_cl，
       已订正 ⇒ 第一版 18 的 C/D 段数字同样作废。但订正侧别后 C/D 仍受 5-point 退化影响，
       见下方【§36 的状态说明】—— Δt_camera 不在本脚本判，改由 19e 判。）

【probe_time 的实测补充，影响本脚本解读】LiDAR 周期 99.8 ms、扫描时长也≈99.8 ms
 ⇒ t_end[k]≈t0[k+1]，A 段的"odom 戳对齐头/中/尾"三选一在戳上不可分辨（同一 100 ms 网格），
   头尾之争是伪问题；真正需要实测的是**图像内容相对其戳的滞后** Δt_camera。

【§36 的状态说明】A/B 段口径正确、结论有效（戳结构 + LIO↔IMU 插值自洽）。
 C/D 段用的是 5-point R5，已实测在低视差下退化 ⇒ 残差绝对值**不得引用**，
 Δt_camera 的权威实测改由 probe_ext3 / 19e_rot_channel_quality（深度辅助模型）给出。

用法：python3 18_timealign_probe.py [pair数]     默认 260
"""
import sys
import time

import numpy as np

sys.path.insert(0, '/home/ywj/git/Super-LIVO/experiments/octvox_ablation')
import _p0_bag as B
import _p0_visual as V

NPAIR = int(sys.argv[1]) if len(sys.argv) > 1 else 260
STEP = 5
BASE = '/home/ywj/slam_data/retail_abl/base.txt'


def imu_attitude(ti, gi):
    """200 Hz 角速度积分成姿态 I(t)（body→起始 body 系），返回可插值 Traj。

    gi 在 body 系 ⇒ Ṙ = R·[ω]× ⇒ 离散递推**右乘** R(k+1)=R(k)·Exp(ω̄Δt)。
    （旧版左乘把 IMU 参照污染 0.454°，B/C/D 三段都受影响 ⇒ 那三段的旧数字作废。）
    """
    Rs = np.empty((len(ti), 3, 3))
    Rs[0] = np.eye(3)
    dt = np.diff(ti)
    for k in range(len(dt)):
        w = 0.5 * (gi[k] + gi[k + 1]) * dt[k]
        Rs[k + 1] = Rs[k] @ V.exp_so3(w)
    q = np.array([V.rot_to_quat_w(r) for r in Rs])
    return V.Traj(ti, np.zeros((len(ti), 3)), q)


def b2c(L):
    """body 系区间相对旋转 -> 相机系：conj_B(L) = R_cl · L · R_clᵗ（R_cl 实测是 b→c）。"""
    return V.rotm(V.R_CL @ L @ V.R_CL.T)


def rel(traj, t0, t1):
    """连续轨迹的区间增量旋转 R_{t0->t1} = A(t1)ᵀ A(t0)；越界返回 None。"""
    _, R0, v0 = traj.at([t0])
    _, R1, v1 = traj.at([t1])
    if not (v0[0] and v1[0]):
        return None
    return R1[0].T @ R0[0]


def main():
    t_start = time.time()
    tl, te, _ = B.read_lidar_stamps(stride=1)
    tm = B.read_image_stamps()
    ti, gi, _ = B.read_imu()
    tum = np.loadtxt(BASE)
    lio = V.Traj(tum[:, 0], tum[:, 1:4], tum[:, 4:8])

    print('===== A. 戳结构 =====')
    print('image.header.stamp 与 lidar.header.stamp 逐帧相等: %s   (n=%d)'
          % (bool(np.array_equal(tm, tl)), len(tm)))
    sl = te - tl
    print('LiDAR 扫描时长 med=%.2f ms  p10=%.2f  p90=%.2f  (>0.2 s 异常 %d 处)'
          % (np.median(sl) * 1e3, np.percentile(sl, 10) * 1e3,
             np.percentile(sl, 90) * 1e3, int((sl > 0.2).sum())))
    for lbl, ref in (('lidar t0', tl), ('lidar t_mid', 0.5 * (tl + te)), ('lidar t_end', te)):
        j = np.clip(np.searchsorted(ref, tum[:, 0]), 1, len(ref) - 1)
        lo, hi = j - 1, j
        best = np.where(np.abs(tum[:, 0] - ref[lo]) < np.abs(tum[:, 0] - ref[hi]), lo, hi)
        err = (tum[:, 0] - ref[best]) * 1e3
        print('   odom 戳 − %-12s : med=%+7.2f ms std=%5.2f min=%+7.2f max=%+7.2f uniq=%d'
              % (lbl, np.median(err), err.std(), err.min(), err.max(), len(set(best.tolist()))))

    idx = [k for k in range(0, len(tm) - STEP, STEP)][:NPAIR]
    idx = [k for k in idx if tl[k + 1] - tl[k] < 0.15]
    print('\n===== 取样 %d 对图像（步长 %d 帧，覆盖 %.0f s）====='
          % (len(idx), STEP, tl[idx[-1]] - tl[idx[0]]))
    t1 = time.time()
    want = sorted(set(idx) | set(k + 1 for k in idx))
    frames = B.read_images(want)
    gray = {k: V.prep_gray(frames[k][1]) for k in want}
    print('读图+去畸变 %d 张：%.1f s' % (len(want), time.time() - t1))

    vis, tk = [], time.time()
    for k in idx:
        d = V.rel_rotation(gray[k], gray[k + 1])
        if d['R5'] is not None and d['n_inl'] >= 40 and d['inlier_ratio'] >= 0.40:
            vis.append((k, d))
    print('有效视觉对 %d / %d；前端耗时 %.1f s（%.0f ms/对）'
          % (len(vis), len(idx), time.time() - tk, (time.time() - tk) / max(1, len(idx)) * 1e3))

    print('\n构建 IMU 姿态积分 ...', end='', flush=True)
    tk = time.time()
    imu = imu_attitude(ti, gi)
    print(' %.1f s（%d 样本）' % (time.time() - tk, len(ti)))
    dt_iv = float(np.median(np.diff(tl)))
    print('区间长度 Δt = %.2f ms（LiDAR 帧周期）' % (dt_iv * 1e3))

    def scan(label, f, grid):
        res = np.array([f(d) for d in grid])
        i = int(np.nanargmin(res))
        print('%s 最优 Δ = %+7.1f ms  中值残差 = %.3f°   '
              '[Δ=0: %.3f° | Δ=+50: %.3f° | Δ=+100: %.3f°]'
              % (label, grid[i] * 1e3, res[i], res[np.argmin(np.abs(grid))],
                 res[np.argmin(np.abs(grid - 0.05))], res[np.argmin(np.abs(grid - 0.10))]))
        return grid[i], res[i], grid, res

    print('\n===== B. LIO 连续轨迹 vs IMU 积分（戳语义/插值自洽性） =====')
    grid = np.arange(-0.15, 0.151, 0.01)

    def resid_b(a):
        e = []
        for k in idx:
            t0, t1x = tl[k] + a, tl[k] + a + dt_iv
            Rl, Ri = rel(lio, t0, t1x), rel(imu, t0, t1x)
            if Rl is not None and Ri is not None:
                e.append(V.angle_of(Rl.T @ Ri))
        return np.median(e) if e else np.nan

    dB, rB, gb, rrb = scan('  [B]', resid_b, grid)
    print('   曲线(deg): ' + '  '.join('%+.0f:%.3f' % (a * 1e3, r)
                                        for a, r in zip(gb[::2], rrb[::2])))

    print('\n===== C. 图像相对旋转 vs IMU 积分（反解 Δt_camera） =====')
    print('   ⚠ 本段用 5-point 的 v[R5]：§36 已实测它在 |t|≈0.06 m 的低视差下退化'
          '（残差 1.6~2°），故 C/D 的**残差绝对值不可用作通道质量判据**，'
          'Δt_camera 以 probe_ext3(n=599) 与 19e(n=500) 的**深度辅助模型**为准。'
          '此处只看极小点的相位是否与那两者一致。')
    grid = np.arange(-0.15, 0.151, 0.005)

    def resid_c(dtc):
        e = []
        for k, v in vis:
            t0, t1x = tl[k] + dtc, tl[k] + dtc + dt_iv
            Ri = rel(imu, t0, t1x)
            if Ri is None:
                continue
            e.append(V.angle_of(v['R5'].T @ b2c(Ri)))
        return np.median(e) if e else np.nan

    dC, rC, gc, rrc = scan('  [C]', resid_c, grid)
    print('   曲线(deg): ' + '  '.join('%+.0f:%.3f' % (a * 1e3, r)
                                        for a, r in zip(gc[::4], rrc[::4])))

    print('\n===== D. 图像相对旋转 vs LIO 插值相对旋转 =====')
    for dtc in (0.0, 0.05, 0.097, dC):
        e = []
        for k, v in vis:
            t0, t1x = tl[k] + dtc, tl[k] + dtc + dt_iv
            Rl = rel(lio, t0, t1x)
            if Rl is None:
                continue
            e.append(V.angle_of(v['R5'].T @ b2c(Rl)))
        e = np.array(e)
        print('   Δ=%+6.1f ms : 残差 med=%.3f° p75=%.3f p90=%.3f  <0.3° 占 %.1f%%'
              % (dtc * 1e3, np.median(e), np.percentile(e, 75), np.percentile(e, 90),
                 100 * (e < 0.3).mean()))

    print('\n===== P0-A 结论 =====')
    print('Δt_camera（图像 vs IMU）   = %+.1f ms（残差 %.3f°）' % (dC * 1e3, rC))
    print('LIO 轨迹 vs IMU 最优偏置   = %+.1f ms（残差 %.3f°）' % (dB * 1e3, rB))
    print('总耗时 %.0f s' % (time.time() - t_start))


if __name__ == '__main__':
    main()
