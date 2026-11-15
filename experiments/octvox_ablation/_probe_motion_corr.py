#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中段误差归因（第七步）：30 桶时间剖面上，误差与"角速度"的相关性。

已排除：位姿分歧、Δt 型平移残差（段内无 1/z 放大）、场景亮度、局部对比度。
剩下的候选是"旋转"相关项：旋转引起的像素错位 ≈ f·θ，与距离无关（正好是观测到的平坦 range 曲线），
且随时间平滑变化（正好是观测到的 30 桶平滑升降）。

MAE 值取自 _probe_time_profile.py 的 30 桶输出（同一 CASE PCD、同一采样种子口径）。

用法: python3 _probe_motion_corr.py
"""
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = "/home/ywj/slam_data/official_vio_20261114/bag"
NBIN = 30
SPAN = 135.47

MAE_ALL = [7.48, 7.46, 7.93, 7.17, 8.06, 8.66, 8.69, 9.27, 9.48, 10.18,
           11.31, 10.25, 11.01, 12.27, 11.21, 11.39, 9.78, 11.42, 11.74, 11.24,
           11.94, 10.24, 10.22, 11.93, 10.64, 10.78, 8.96, 8.93, 9.80, 9.18]
MAE_NEAR = [7.08, 6.95, 6.84, np.nan, 7.51, 8.38, 7.57, 9.00, 10.22, 10.68,
            14.24, 13.17, 13.55, 13.63, 15.19, 16.54, 16.89, 16.37, 15.88, 15.51,
            12.97, 12.04, 12.78, 12.87, 10.99, 9.93, 7.58, 7.88, 8.12, 8.40]
BAD30 = [2.0, 2.0, 3.7, 3.2, 3.8, 4.0, 3.8, 4.3, 5.2, 6.0,
         7.8, 6.4, 7.5, 9.0, 8.1, 8.2, 6.9, 8.1, 8.4, 7.9,
         8.2, 6.2, 6.2, 8.3, 7.6, 6.5, 3.9, 4.2, 4.1, 2.7]


def main():
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/aft_mapped_lidar_to_init"]))
    typ = get_message("nav_msgs/msg/Odometry")
    ts, P, Q = [], [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, typ)
        ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        P.append([p.x, p.y, p.z])
        Q.append([q.x, q.y, q.z, q.w])
    ts, P, Q = np.array(ts), np.array(P), np.array(Q)
    t = ts - ts[0]

    def qmul(a, b):
        ax, ay, az, aw = a.T
        bx, by, bz, bw = b.T
        return np.stack([
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz], 1)

    dt = np.diff(ts)
    v = np.linalg.norm(np.diff(P, axis=0), axis=1) / dt
    qinv = Q[:-1] * np.array([-1, -1, -1, 1])
    dq = qmul(qinv, Q[1:])
    ang = 2.0 * np.arccos(np.clip(np.abs(dq[:, 3]), -1.0, 1.0))
    w = np.degrees(ang) / dt
    tm = 0.5 * (t[:-1] + t[1:])

    edges = np.linspace(0, SPAN, NBIN + 1)
    ws, vs = [], []
    for b in range(NBIN):
        m = (tm >= edges[b]) & (tm < edges[b + 1])
        ws.append(np.median(w[m]) if m.any() else np.nan)
        vs.append(np.median(v[m]) if m.any() else np.nan)
    ws, vs = np.array(ws), np.array(vs)

    print("桶  t     w(p50°/s) v(p50 m/s)  MAE_all  MAE_near  >30%")
    for b in range(NBIN):
        print("%2d %5.1f  %8.1f  %9.2f  %7.2f  %8s  %4.1f" % (
            b, edges[b], ws[b], vs[b], MAE_ALL[b],
            "-" if np.isnan(MAE_NEAR[b]) else "%.2f" % MAE_NEAR[b], BAD30[b]))

    ma = np.array(MAE_ALL)
    mn = np.array(MAE_NEAR)
    bad = np.array(BAD30)
    print("\n相关系数（30 桶）：")
    print("  MAE_all  vs w      = %.3f" % np.corrcoef(ma, ws)[0, 1])
    print("  MAE_all  vs v      = %.3f" % np.corrcoef(ma, vs)[0, 1])
    print("  MAE_all  vs w*v    = %.3f" % np.corrcoef(ma, ws * vs)[0, 1])
    print("  MAE_all  vs w+v    = %.3f" % np.corrcoef(ma, ws + vs)[0, 1])
    ok = ~np.isnan(mn)
    print("  MAE_near vs w      = %.3f" % np.corrcoef(mn[ok], ws[ok])[0, 1])
    print("  MAE_near vs v      = %.3f" % np.corrcoef(mn[ok], vs[ok])[0, 1])
    print("  MAE_near vs w*v    = %.3f" % np.corrcoef(mn[ok], (ws * vs)[ok])[0, 1])
    print("  >30%%     vs w      = %.3f" % np.corrcoef(bad, ws)[0, 1])
    print("  >30%%     vs v      = %.3f" % np.corrcoef(bad, vs)[0, 1])
    print("DONE_MOTION_CORR")


if __name__ == "__main__":
    main()
