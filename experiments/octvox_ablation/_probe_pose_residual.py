#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中段位姿残差诊断：从录制流里读出 LIO 与 VIO 两条位姿序列，量化它们的分歧。

着色链路的双位姿口径：
  点云世界坐标 = T_lio(t_cloud) × p_lidar
  相机位姿     = T_vio(t_img)，t_img = header + 0.1
若 T_lio 与 T_vio 在某一时段分歧变大，点云就会被投到错误的像素上 —— 既压低
几何同位率，也直接抬高色差。这里把分歧 D(t) = T_lio^-1 · T_vio 归一化到起始时刻，
看平移/旋转随时间的漂移，并按 PCD 落盘顺序的时间 6 等分对齐到之前的段表。

用法: python3 _probe_pose_residual.py [bag_dir]
"""
import sys
import numpy as np

BAG = "/home/ywj/slam_data/official_vio_20261114/bag"
if len(sys.argv) > 1:
    BAG = sys.argv[1]

LIO_TOPIC = "/aft_mapped_lidar_to_init"
VIO_TOPIC = "/vio/odom_lidar"
SEGS = 6

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quat_to_R(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def read_poses(topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))

    out = {t: ([], []) for t in topics}
    types = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in types:
            types[topic] = get_message(
                {t: "nav_msgs/msg/Odometry" for t in topics}[topic])
        msg = deserialize_message(data, types[topic])
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        out[topic][0].append(st)
        out[topic][1].append(([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
    res = {}
    for t in topics:
        ts = np.array(out[t][0])
        order = np.argsort(ts)
        ts = ts[order]
        P = np.array([out[t][1][i][0] for i in order])
        Q = np.array([out[t][1][i][1] for i in order])
        res[t] = (ts, P, Q)
        print("%s: n=%d  span=%.2fs  rate=%.2fHz" % (
            t, len(ts), ts[-1] - ts[0], (len(ts) - 1) / (ts[-1] - ts[0])))
    return res


def interp_pose(ts, P, Q, tq):
    """线性插值平移 + 四元数半球对齐后线性插值（足够做残差诊断）。"""
    idx = np.searchsorted(ts, tq, side="right").clip(1, len(ts) - 1)
    t0, t1 = ts[idx - 1], ts[idx]
    w = ((tq - t0) / np.maximum(t1 - t0, 1e-9))[:, None]
    p = P[idx - 1] * (1 - w) + P[idx] * w
    q0, q1 = Q[idx - 1], Q[idx]
    sign = np.sign((q0 * q1).sum(1))[:, None]
    q1 = q1 * np.where(sign == 0, 1.0, sign)
    q = q0 * (1 - w) + q1 * w
    q /= np.linalg.norm(q, axis=1)[:, None]
    return p, q


def inv_T(p, q):
    R = quat_to_R(q)
    Rt = R.T
    return Rt, -Rt @ p


def rot_angle_deg(R):
    c = (np.trace(R) - 1.0) * 0.5
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def main():
    data = read_poses([LIO_TOPIC, VIO_TOPIC])
    tl, Pl, Ql = data[LIO_TOPIC]
    tv, Pv, Qv = data[VIO_TOPIC]

    tq = tl.copy()
    pv, qv = interp_pose(tv, Pv, Qv, tq)

    # D(t) = T_lio^-1 T_vio ；再以首帧归一化，抵消两个世界系之间的固定外参差
    trans, rot = [], []
    for i in range(len(tq)):
        Rl, tl_ = inv_T(Pl[i], Ql[i])
        Rv = quat_to_R(qv[i])
        dR = Rl @ Rv
        dt = Rl @ pv[i] + tl_
        trans.append(np.linalg.norm(dt))
        if i == 0:
            R0, t0 = dR.copy(), dt.copy()
        else:
            trans[-1] = np.linalg.norm(dt - R0.T @ t0)
            rot.append(rot_angle_deg(R0.T @ dR))
    trans = np.array(trans[1:])
    rot = np.array(rot)
    tq = tq[1:]
    print("\nD(t)=T_lio^-1·T_vio（以首帧归一化）:")
    print("  平移 p50=%.3fm p90=%.3fm max=%.3fm" % (
        np.percentile(trans, 50), np.percentile(trans, 90), trans.max()))
    print("  旋转 p50=%.2f° p90=%.2f° max=%.2f°" % (
        np.percentile(rot, 50), np.percentile(rot, 90), rot.max()))

    # 两条轨迹各自的运动速度（判断中段是否高动态/是否出现跳变）
    def speeds(ts, P):
        d = np.diff(P, axis=0)
        dt = np.maximum(np.diff(ts), 1e-9)
        return np.linalg.norm(d, axis=1) / dt

    sl, sv = speeds(tl, Pl), speeds(tv, Pv)
    print("LIO 速度 p50=%.2fm/s p95=%.2fm/s max=%.2fm/s | VIO 速度 p50=%.2fm/s p95=%.2fm/s max=%.2fm/s" % (
        np.percentile(sl, 50), np.percentile(sl, 95), sl.max(),
        np.percentile(sv, 50), np.percentile(sv, 95), sv.max()))

    # 按时间 6 等分（≈ PCD 落盘顺序 6 段）
    print("\n按时间分段（≈ PCD 段的先后顺序）:")
    print("段   t起(s)   平移中位 平移p90  旋转中位 旋转p90 | LIO速度p50 VIO速度p50 | 路径长(m)")
    edges = np.linspace(tq[0], tq[-1], SEGS + 1)
    for s in range(SEGS):
        m = (tq >= edges[s]) & (tq < edges[s + 1] if s < SEGS - 1 else tq <= edges[s + 1])
        mv = (tv >= edges[s]) & (tv < edges[s + 1] if s < SEGS - 1 else tv <= edges[s + 1])
        ml = (tl >= edges[s]) & (tl < edges[s + 1] if s < SEGS - 1 else tl <= edges[s + 1])
        if np.count_nonzero(m) < 3:
            continue
        pl = np.linalg.norm(np.diff(Pl[ml], axis=0), axis=1).sum() if np.count_nonzero(ml) > 1 else 0
        print("%d  %7.2f  %7.3f %7.3f  %7.2f %7.2f | %8.2f %8.2f | %7.1f" % (
            s, edges[s],
            np.median(trans[m]), np.percentile(trans[m], 90),
            np.median(rot[m]), np.percentile(rot[m], 90),
            np.median(sl[ml[:-1]]) if np.count_nonzero(ml) > 1 else float("nan"),
            np.median(sv[mv[:-1]]) if np.count_nonzero(mv) > 1 else float("nan"),
            pl))

    # 分歧增长最快的时刻
    k = np.argsort(trans)[-5:]
    print("\n平移分歧最大的 5 个时刻: " + ", ".join(
        "%.2f(+%.2fs):%.3fm" % (tq[i], tq[i] - tq[0], trans[i]) for i in k))
    k = np.argsort(rot)[-5:]
    print("旋转分歧最大的 5 个时刻: " + ", ".join(
        "%.2f(+%.2fs):%.2f°" % (tq[i], tq[i] - tq[0], rot[i]) for i in k))
    print("DONE_POSE_RESIDUAL")


if __name__ == "__main__":
    main()
