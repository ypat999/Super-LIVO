#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按时间 6 等分统计 LIO 的线速度/角速度，用于检验"近距离色差抬升 = 运动 × 时间对齐残差"。

近距离点的像素位移 ≈ f · v⊥ · Δt / z（z 小 → 放大），若中段线速度/角速度更高，
则同样的 img_time_offset 残差会在近处产生明显更大的色差。

用法: python3 _probe_seg_motion.py
"""
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = "/home/ywj/slam_data/official_vio_20261114/bag"
SEGS = 6


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

    print("整体: v p50=%.2f m/s p90=%.2f | w p50=%.1f °/s p90=%.1f" % (
        np.median(v), np.percentile(v, 90), np.median(w), np.percentile(w, 90)))
    print("\n段  t起(s)   v_p50  v_p90  w_p50  w_p90   | 近距(0-6m)MAE(前一步)")
    near = [6.99, 9.15, 13.92, 15.68, 12.28, 8.34]
    edges = np.linspace(ts[0], ts[-1], SEGS + 1)
    seg_v, seg_w = [], []
    for s in range(SEGS):
        m = (ts[:-1] >= edges[s]) & (ts[:-1] < edges[s + 1])
        seg_v.append(np.median(v[m]))
        seg_w.append(np.median(w[m]))
        print("%d  %7.2f  %5.2f  %5.2f  %5.1f  %5.1f   |  %5.2f" % (
            s, edges[s], np.median(v[m]), np.percentile(v[m], 90),
            np.median(w[m]), np.percentile(w[m], 90), near[s]))
    seg_v, seg_w, near = map(np.array, (seg_v, seg_w, near))
    print("\n相关系数: 近距MAE vs v=%.3f  vs w=%.3f  vs (v+w)=%.3f" % (
        np.corrcoef(near, seg_v)[0, 1], np.corrcoef(near, seg_w)[0, 1],
        np.corrcoef(near, seg_v + seg_w)[0, 1]))
    print("DONE_SEG_MOTION")


if __name__ == "__main__":
    main()
