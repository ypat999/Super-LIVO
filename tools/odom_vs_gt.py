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
"""
import argparse
import os
import sys

import numpy as np

# 官方 body->prism 平移（m）
T_B_PRISM = np.array([-0.293656, -0.012288, -0.273095])
MAX_GAP_S = 0.1


def load_gt(path):
    """NTU VIRAL ground_truth.csv：列 = time_ns, seq, header_stamp_ns, x,y,z, qx,qy,qz,qw"""
    rows = []
    with open(path) as f:
        header = f.readline().strip().lstrip('%').split(',')
        for line in f:
            p = line.strip().split(',')
            if len(p) < 7:
                continue
            try:
                t = float(p[0]) * 1e-9
                rows.append([t] + [float(x) for x in p[3:7]])
            except ValueError:
                continue
    a = np.array(rows)
    if len(a) == 0:
        sys.exit('真值文件为空或格式不符: %s（表头=%s）' % (path, header[:6]))
    a = a[np.argsort(a[:, 0])]
    print('真值: %s  n=%d  时长 %.1f s' % (os.path.basename(path), len(a), a[-1, 0] - a[0, 0]))
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


def evaluate(label, est, gt):
    """est: (N,8) [t,x,y,z,qx,qy,qz,qw]；gt: (M,4) [t,x,y,z]"""
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
    return dict(label=label, ate=ate, err=err, t=te, pe=pe_a, pg=pg, qs=qs)


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
    a = ap.parse_args()

    if len(a.topic) < len(a.bag):
        a.topic += ['/lio/odom'] * (len(a.bag) - len(a.topic))
    gt = load_gt(a.gt)
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
        r = evaluate(lab, est, gt)
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
