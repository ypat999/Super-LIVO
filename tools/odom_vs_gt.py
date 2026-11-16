#!/usr/bin/env python3
"""NTU VIRAL 里程计精度评测（严格按官方 evaluation_tutorial 协议实现）。

协议要点（https://ntu-aris.github.io/ntu_viral_dataset/evaluation_tutorial.html）：
  1. 真值是棱镜位置，估计是 IMU 体中心 → 先把体→棱镜外参加到估计上：
         P_est += R_est · T_B_prism
     T_B_prism = (-0.293656, -0.012288, -0.273095)
  2. 重采样：对每个估计样本，用其前后两个真值样本内插；若任一侧间隔 > 0.1 s 则丢弃该样本
     （对应官方 combteeth(t_est, t, 0.1)）。
  3. 对齐：对位置做 SE(3) Umeyama（不估尺度），取最小 RMSE 的那个变换。
  4. 指标：ATE = 对齐后位置误差的 RMSE。本脚本另加分位时间段的 ATE，用来显示漂移累积。

用法：
  odom_vs_gt.py --gt gt/rtp_01/ground_truth.csv \
                --bag  <rosbag2目录> --topic /lio/odom \
                [--label 名称] [--out png路径]
  可同时给多组：--bag A --topic /lio/odom --bag B --topic /super_livo_visual_a/odom ...

旋转通道（与 exp_records §36.11 的 0.1 s 区间口径对齐）：
  NTU VIRAL 的真值来自全站仪棱镜，**只有位置没有姿态**（四元数列恒为 (0,0,0,1)），
  所以位置 ATE 无法约束旋转。要量旋转必须另找参照：
    --imu-topic /imu/imu --imu-bag <bag目录>
  用陀螺积分出同一区间的旋转 ΔR_imu，与里程计区间旋转 ΔR_est 比较：
      e = log( (R̂ ΔR_imu R̂ᵀ)ᵀ ΔR_est )
  R̂ 是 IMU↔odom 的常值旋转（Kabsch 拟合掉外参/轴系差异），逐轴去中位消掉陀螺零偏，
  剩下的 |e| 才是区间旋转误差。同时扫描时间偏移 δ（雷达/IMU 时钟对齐误差会伪装成旋转误差）。
  真值若带姿态（std > 1e-4）则额外给 GT 参照的旋转 RPE。
"""
import argparse
import os
import sys

import numpy as np

# 官方 body->prism 平移（m）
T_B_PRISM = np.array([-0.293656, -0.012288, -0.273095])
MAX_GAP_S = 0.1


def load_gt(path):
    """NTU VIRAL ground_truth.csv：列 = time_ns, seq, header_stamp_ns, x,y,z, qx,qy,qz,qw

    返回 (M,8)：[t, x, y, z, qx, qy, qz, qw]。NTU 的四元数列恒为单位四元数（无姿态），
    保留它是为了让下游能显式检查并跳过 GT 旋转通道。
    """
    rows = []
    with open(path) as f:
        header = f.readline().strip().lstrip('%').split(',')
        for line in f:
            p = line.strip().split(',')
            if len(p) < 7:
                continue
            try:
                t = float(p[0]) * 1e-9
                vals = [float(x) for x in p[3:7]]
                q = [float(x) for x in p[7:11]] if len(p) >= 11 else [0.0, 0.0, 0.0, 1.0]
                rows.append([t] + vals + q)
            except ValueError:
                continue
    a = np.array(rows)
    if len(a) == 0:
        sys.exit('真值文件为空或格式不符: %s（表头=%s）' % (path, header[:6]))
    a = a[np.argsort(a[:, 0])]
    qs = a[:, 4:8].std(axis=0).max() if len(a) > 2 else 0.0
    print('真值: %s  n=%d  时长 %.1f s  姿态列%s' %
          (os.path.basename(path), len(a), a[-1, 0] - a[0, 0],
           '有效(std=%.3g)' % qs if qs > 1e-4 else '恒定（无姿态真值，旋转只能走 IMU 参照）'))
    return a



def load_odom(bag_dir, topic, time_source='header'):
    """纯 Python 读 ROS2 bag（不需要 source ROS 环境）。"""
    if bag_dir.endswith('.txt'):
        # 官方 FAST-LIVO2 的 Log/result/<seq>.txt：TUM 格式 t x y z qx qy qz qw（时间为数据时间）
        rows = []
        for line in open(bag_dir):
            p = line.split()
            if len(p) < 8:
                continue
            rows.append([float(x) for x in p[:8]])
        a = np.array(rows)
        print('  官方轨迹文件 %s  n=%d' % (os.path.basename(bag_dir), len(a)))
        return a[np.argsort(a[:, 0])] if len(a) >= 10 else None
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS2_HUMBLE)
    out = []
    with Reader(bag_dir) as r:
        topics = {c.topic for c in r.connections}
        if topic not in topics:
            print('  话题缺失 %s；该 bag 有: %s' % (topic, sorted(topics)))
            return None
        conns = [c for c in r.connections if c.topic == topic]
        for conn, t_ns, raw in r.messages(connections=conns):
            msg = ts.deserialize_cdr(raw, conn.msgtype)
            if time_source == 'header':
                st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            else:
                st = t_ns * 1e-9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            out.append([st, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    a = np.array(out)
    if len(a) < 10:
        print('  %s 样本过少(%d)' % (topic, len(a)))
        return None
    return a[np.argsort(a[:, 0])]


def quat_to_rot(q):
    x, y, z, w = q
    n = np.linalg.norm([x, y, z, w])
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (y * y + x * x)]])


def interp_gt_at(gt, t_est):
    """官方重采样规则：只用前后真值样本内插，任一侧间隔 > MAX_GAP_S 则丢弃。"""
    tg, pg = gt[:, 0], gt[:, 1:4]
    idx = np.searchsorted(tg, t_est)
    i_hi = np.clip(idx, 1, len(tg) - 1)
    i_lo = i_hi - 1
    dt_lo = t_est - tg[i_lo]
    dt_hi = tg[i_hi] - t_est
    ok = (dt_lo >= -MAX_GAP_S) & (dt_hi >= -MAX_GAP_S) & \
         (np.abs(dt_lo) <= MAX_GAP_S) & (np.abs(dt_hi) <= MAX_GAP_S)
    span = tg[i_hi] - tg[i_lo]
    w = np.where(np.abs(span) > 1e-12, (t_est - tg[i_lo]) / np.where(span == 0, 1, span), 0.0)
    p = pg[i_lo] * (1 - w)[:, None] + pg[i_hi] * w[:, None]
    return p, ok


def umeyama_se3(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    C = (dst - mu_d).T @ (src - mu_s) / len(src)
    U, _, Vt = np.linalg.svd(C)
    d = np.sign(np.linalg.det(U @ Vt))
    R = U @ np.diag([1, 1, d]) @ Vt
    t = mu_d - R @ mu_s
    return R, t


# ===================== 旋转区间 RPE（IMU 参照 / GT 参照） =====================

def load_imu(bag_dir, topic):
    """读 sensor_msgs/Imu 的角速度 → (N,4) [t, gx, gy, gz]（t 取 header.stamp）。"""
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS2_HUMBLE)
    out = []
    with Reader(bag_dir) as r:
        topics = {c.topic for c in r.connections}
        if topic not in topics:
            print('  IMU 话题缺失 %s；该 bag 有: %s' % (topic, sorted(topics)))
            return None
        conns = [c for c in r.connections if c.topic == topic]
        for conn, _, raw in r.messages(connections=conns):
            m = ts.deserialize_cdr(raw, conn.msgtype)
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            w = m.angular_velocity
            out.append([st, w.x, w.y, w.z])
    a = np.array(out)
    if len(a) < 100:
        print('  IMU 样本过少(%d)' % len(a))
        return None
    a = a[np.argsort(a[:, 0])]
    dt = np.diff(a[:, 0])
    print('IMU: %s   n=%d  时长 %.1f s   间隔 P50 %.1f ms  >50ms 空洞 %d' %
          (topic, len(a), a[-1, 0] - a[0, 0], np.median(dt) * 1e3, int((dt > 0.05).sum())))
    return a


def exp_so3(w):
    th = float(np.linalg.norm(w))
    K = np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])
    if th < 1e-10:
        return np.eye(3) + K + 0.5 * K @ K
    return np.eye(3) + (np.sin(th) / th) * K + ((1.0 - np.cos(th)) / th ** 2) * (K @ K)


def log_so3(R):
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        R = U @ np.diag([1.0, 1.0, -1.0]) @ Vt
    c = np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)
    th = float(np.arccos(c))
    if th < 1e-9:
        return np.zeros(3)
    if np.pi - th < 1e-6:                       # 180° 附近 sinθ→0，用对称部分取轴
        A = 0.5 * (R + np.eye(3))
        k = np.sqrt(np.maximum(np.diag(A), 0.0))
        j = int(np.argmax(k))
        if k[j] > 0:
            k = A[:, j] / k[j]
        k = k / max(np.linalg.norm(k), 1e-12)
        return th * k
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w * (th / (2.0 * np.sin(th)))


def integrate_gyro(imu, t0, t1):
    """陀螺积分 [t0,t1] → SO(3)（body 系增量旋转）。端点线性内插，梯形积分。"""
    if imu is None or t1 <= t0:
        return None
    if t0 < imu[0, 0] - 1e-3 or t1 > imu[-1, 0] + 1e-3:
        return None
    i0, i1 = np.searchsorted(imu[:, 0], [t0, t1])
    ts = np.concatenate(([t0], imu[i0:i1, 0], [t1]))
    w0 = np.array([np.interp(t0, imu[:, 0], imu[:, k]) for k in (1, 2, 3)])
    w1 = np.array([np.interp(t1, imu[:, 0], imu[:, k]) for k in (1, 2, 3)])
    w = np.vstack((w0, imu[i0:i1, 1:4], w1))
    R = np.eye(3)
    for j in range(len(ts) - 1):
        dt = ts[j + 1] - ts[j]
        if dt <= 0:
            continue
        if dt > 0.05:                           # 空洞，跳过该段（避免一次误差污染整段）
            continue
        R = R @ exp_so3(0.5 * (w[j] + w[j + 1]) * dt)
    return R


def pairs_at_dt(t, dt, tol=0.3):
    """在时间序列上取固定基线 dt 的配对 (i, j)，j 时刻最接近 t_i+dt。"""
    n = len(t)
    idx = np.searchsorted(t, t + dt)
    cand = np.clip(idx, 1, n - 1)
    alt = cand - 1
    use_alt = np.abs(t[alt] - (t + dt)) < np.abs(t[cand] - (t + dt))
    j = np.where(use_alt, alt, cand)
    ok = (j > np.arange(n)) & (np.abs(t[j] - t - dt) <= tol * dt)
    i = np.where(ok)[0]
    return i, j[i]


def fit_const_rot(a_vec, b_vec):
    """Kabsch：最小化 Σ‖R a_i − b_i‖²，R 把 IMU 区间旋转向量对齐到 odom 区间旋转向量。"""
    if len(a_vec) < 10:
        return np.eye(3)
    C = b_vec.T @ a_vec
    U, _, Vt = np.linalg.svd(C)
    d = np.sign(np.linalg.det(U @ Vt))
    return U @ np.diag([1.0, 1.0, d]) @ Vt


def _rot_stats(e, tag):
    """e: (N,3) 弧度误差（已去常值）。返回 dict 并打印。"""
    if len(e) == 0:
        print('    旋转 %s: 无可用样本' % tag)
        return None
    n = np.linalg.norm(e, axis=1)
    ez = np.abs(e[:, 2])
    d = dict(n=len(e), rms=float(np.degrees(np.sqrt(np.mean(n ** 2)))),
             med=float(np.degrees(np.median(n))),
             p90=float(np.degrees(np.percentile(n, 90))),
             yaw_rms=float(np.degrees(np.sqrt(np.mean(ez ** 2)))),
             yaw_med=float(np.degrees(np.median(ez))))
    print('    %s n=%4d  |e| RMS=%.4f° 中位=%.4f° P90=%.4f°   |e_z|(偏航) RMS=%.4f° 中位=%.4f°' %
          (tag, d['n'], d['rms'], d['med'], d['p90'], d['yaw_rms'], d['yaw_med']))
    return d


def rotation_interval_rpe(label, est, gt, imu, dts,
                          scan_offsets=(-0.02, -0.01, 0.0, 0.01, 0.02)):
    """区间旋转误差：est 自身区间旋转 vs 参照（IMU 积分 / GT 姿态）。"""
    t = est[:, 0]
    R = np.array([quat_to_rot(q) for q in est[:, 4:8]])
    print('%-28s 旋转区间 RPE' % label)
    best = 0.0
    gt_has_att = (gt is not None and gt.shape[1] >= 8 and
                  float(np.std(gt[:, 4:8], axis=0).max()) > 1e-4)
    for dt in dts:
        i, j = pairs_at_dt(t, dt)
        if len(i) < 20:
            print('    Δt=%.2f s: 配对过少(%d)' % (dt, len(i)))
            continue
        dRe = np.array([log_so3(R[ii].T @ R[jj]) for ii, jj in zip(i, j)])
        if imu is not None:
            dRi = [integrate_gyro(imu, t[ii], t[jj]) for ii, jj in zip(i, j)]
            keep = np.array([x is not None for x in dRi])
            if keep.sum() < 20:
                print('    Δt=%.2f s: IMU 可用区间过少' % dt)
                continue
            vi = np.array([log_so3(x) for x, k in zip(dRi, keep) if k])
            ve = dRe[keep]
            Ra = fit_const_rot(vi, ve)
            ang = np.degrees(np.linalg.norm(log_so3(Ra)))
            e0 = np.array([log_so3(exp_so3(a).T @ exp_so3(b)) for a, b in zip(vi, ve)])
            e = np.array([log_so3(exp_so3(Ra @ a).T @ exp_so3(b)) for a, b in zip(vi, ve)])
            e = e - np.median(e, axis=0)         # 逐轴去中位：陀螺零偏 + 常值轴偏差
            print('  Δt=%.2f s  IMU↔odom 常值旋转 %.3f°（未拟合 |e| RMS %.4f°）' %
                  (dt, ang, np.degrees(np.sqrt(np.mean(np.linalg.norm(e0, axis=1) ** 2)))))
            st = _rot_stats(e, 'IMU 参照')
            if st:
                best = max(best, st['rms'])
            # 时间偏移扫描：δ 同时平移两端，看误差谷底在哪（时钟错位会伪装成旋转误差）
            scan = []
            for off in scan_offsets:
                dRo = [integrate_gyro(imu, t[ii] + off, t[jj] + off) for ii, jj in zip(i, j)]
                kp = np.array([x is not None for x in dRo])
                if kp.sum() < 20:
                    scan.append((off, float('nan')))
                    continue
                vo = np.array([log_so3(x) for x, k in zip(dRo, kp) if k])
                eo = np.array([log_so3(exp_so3(a).T @ exp_so3(b)) for a, b in zip(vo, dRe[kp])])
                eo = eo - np.median(eo, axis=0)
                scan.append((off, float(np.degrees(np.median(np.linalg.norm(eo, axis=1))))))
            print('    时间偏移扫描(中位°, δ=秒): ' +
                  '  '.join('%+.3f:%.3f' % (o, v) for o, v in scan))
        if gt_has_att:
            tg, Rg = gt[:, 0], np.array([quat_to_rot(q) for q in gt[:, 4:8]])
            # GT 姿态按最近样本（GT 通常 100 Hz，与 odom 采样不同步；间隔 >20 ms 的配对丢弃）
            jj = np.searchsorted(tg, t[j])
            okj = (jj > 0) & (jj < len(tg)) & (np.abs(tg[np.clip(jj, 0, len(tg) - 1)] - t[j]) < 0.02)
            ii = np.searchsorted(tg, t[i])
            oki = (ii > 0) & (ii < len(tg)) & (np.abs(tg[np.clip(ii, 0, len(tg) - 1)] - t[i]) < 0.02)
            keep = okj & oki
            if keep.sum() >= 20:
                dRg = np.array([log_so3(Rg[a].T @ Rg[b]) for a, b in
                                zip(ii[keep], jj[keep])])
                eg = np.array([log_so3(exp_so3(a).T @ exp_so3(b)) for a, b in
                               zip(dRg, dRe[keep])])
                eg = eg - np.median(eg, axis=0)
                _rot_stats(eg, 'GT 参照')
    return best


def evaluate(label, est, gt, imu=None, rot_dts=None):
    """est: (N,8) [t,x,y,z,qx,qy,qz,qw]；gt: (M,8) [t,x,y,z,qx,qy,qz,qw]"""
    t = est[:, 0]
    p = est[:, 1:4].copy()
    # 1) 体->棱镜补偿（用估计自身的姿态）
    for i in range(len(p)):
        p[i] += quat_to_rot(est[i, 4:8]) @ T_B_PRISM
    # 2) 重采样到估计时刻
    pg, ok = interp_gt_at(gt, t)
    if ok.sum() < 20:
        print('%-28s 可用样本太少(%d)，真值与估计时间不重叠？' % (label, ok.sum()))
        return None
    pe, pg = p[ok], pg[ok]
    te = t[ok]
    # 3) SE(3) 对齐
    R, tr = umeyama_se3(pe, pg)
    pe_a = (R @ pe.T).T + tr
    err = np.linalg.norm(pe_a - pg, axis=1)
    # 4) 指标
    ate = np.sqrt(np.mean(err ** 2))
    span = te[-1] - te[0]
    qs = []
    for k in range(5):
        m = (te >= te[0] + k * span / 5) & (te < te[0] + (k + 1) * span / 5)
        qs.append(np.sqrt(np.mean(err[m] ** 2)) if m.sum() > 5 else float('nan'))
    print('%-28s n=%5d  ATE(RMSE)=%7.3f m  中位=%6.3f  P95=%7.3f  最大=%8.3f' %
          (label, len(err), ate, np.median(err), np.percentile(err, 95), err.max()))
    print('    分段时间 ATE(m): ' + ' '.join('%.0f-%.0f%%:%7.3f' % (k * 20, (k + 1) * 20, v)
                                        for k, v in enumerate(qs)) +
          '   [总时长 %.0f s]' % span)
    # 5) 旋转通道（位置 ATE 对旋转不敏感，必须单独量）
    rot = rotation_interval_rpe(label, est, gt, imu, rot_dts or [0.1])
    return dict(label=label, ate=ate, err=err, t=te, pe=pe_a, pg=pg, qs=qs, rot_rms=rot)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gt', required=True)
    ap.add_argument('--bag', action='append', required=True)
    ap.add_argument('--topic', action='append', default=[])
    ap.add_argument('--label', action='append', default=[])
    ap.add_argument('--time-source', default='header', choices=['header', 'bag'])
    # 官方 FAST-LIVO2 用 rclcpp::now() 给 odom 打时间戳（墙钟），与 bag 内数据时间不同源。
    # auto = 把估计的首个时刻平移到真值首个时刻；也可直接给常数偏移（秒，加到估计时间上）。
    ap.add_argument('--time-shift', default='')
    ap.add_argument('--out', default='')
    # 旋转通道：真值无姿态时用陀螺积分做参照
    ap.add_argument('--imu-bag', default='', help='IMU 所在 bag 目录（默认与第一个 --bag 相同）')
    ap.add_argument('--imu-topic', default='', help='如 /imu/imu；留空则跳过旋转通道')
    ap.add_argument('--rot-dt', default='0.1', help='旋转区间基线（秒，逗号分隔）')
    a = ap.parse_args()

    if len(a.topic) < len(a.bag):
        a.topic += ['/lio/odom'] * (len(a.bag) - len(a.topic))
    gt = load_gt(a.gt)
    imu = None
    if a.imu_topic:
        imu = load_imu(a.imu_bag or a.bag[0], a.imu_topic)
        if imu is None:
            print('  IMU 载入失败，旋转通道跳过')
    rot_dts = [float(x) for x in a.rot_dt.split(',') if x.strip()]
    res = []
    for i, bag in enumerate(a.bag):
        lab = a.label[i] if i < len(a.label) else '%s %s' % (os.path.basename(bag), a.topic[i])
        est = load_odom(bag, a.topic[i], a.time_source)
        if est is None:
            continue
        if a.time_shift:
            shift = gt[0, 0] - est[0, 0] if a.time_shift == 'auto' else float(a.time_shift)
            est[:, 0] += shift
            print('  时间平移 %+.3f s（对齐到真值基准）' % shift)
        # 时间基准对齐：真值与 odom 的 ns 时钟同源，但 header 可能有常数偏差
        r = evaluate(lab, est, gt, imu, rot_dts)
        if r:
            res.append(r)

    if a.out and res:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(15, 6))
        for r in res:
            axes[0].plot(r['t'] - r['t'][0], r['err'], lw=1.0, label='%s ATE=%.2f m' % (r['label'], r['ate']))
            axes[1].plot(r['pg'][:, 0], r['pg'][:, 1], lw=1.2, alpha=0.5, label='GT')
            axes[1].plot(r['pe'][:, 0], r['pe'][:, 1], lw=1.0, alpha=0.8, label=r['label'])
        axes[0].set_xlabel('elapsed (s)')
        axes[0].set_ylabel('position error after SE(3) align (m)')
        axes[0].legend(fontsize=8)
        axes[0].grid(alpha=0.3)
        axes[1].set_aspect('equal')
        axes[1].legend(fontsize=8)
        axes[1].grid(alpha=0.3)
        axes[1].set_title('trajectory (x-y), SE(3)-aligned to GT')
        fig.tight_layout()
        fig.savefig(a.out, dpi=120)
        print('图已保存:', a.out)


if __name__ == '__main__':
    main()
