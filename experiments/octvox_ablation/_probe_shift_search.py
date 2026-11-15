#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位"单帧投影像素偏移"：对每帧做 ±12px 网格搜索，找使"与官方颜色 MAE 最小"的像素位移。

复刻口径（与 visual_a_node / ColorizerPipeline 一致）：
  p_world = T_wl_cloud(LIO@t_cloud) · p_body
  T_wc    = T_wl_img(VIO@t_img) · T_lc,  T_lc = inv([Rcl|Pcl])      # T_lidar_cam
  p_cam   = inv(T_wc) · p_world
  u = fx·x/z + cx,  v = fy·y/z + cy           （图像先按 K,D 去畸变，内参不变）
  t_img   = image_header + img_time_offset(0.1)

判定：
  * 最优位移在各帧一致且明显非零 → 外参/位姿系统性角度偏差；
  * 最优位移随帧变化、且与角速度同向成比例 → img_time_offset 与旋转耦合。

用法: python3 _probe_shift_search.py
"""
import numpy as np
import cv2
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree

FL2 = "/home/ywj/slam_data/official_vio_20261114/bag"
RAW = "/home/ywj/slam_data/Retail_Street"
REF = "/home/ywj/slam_data/pcd/retail_street_livo_official/retail_street_livo_rgb_v2.pcd"

FX, FY, CX, CY = 646.78472, 646.65775, 313.456795, 261.399612
DIST = np.array([-0.076160, 0.123001, -0.00113, 0.000251])
RCL = np.array([0.00610193, -0.999863, -0.0154172,
                -0.00615449, 0.0153796, -0.999863,
                0.999962, 0.00619598, -0.0060598]).reshape(3, 3)
PCL = np.array([0.0194384, 0.104689, -0.0251952])
IMG_OFFSET = 0.1
SEGS = 6
FRAMES_PER_SEG = 2
NPT = 8000
SHIFT = 12


def q2R(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read_odom(path, topic, is_vio):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    typ = get_message("nav_msgs/msg/Odometry")
    ts, P, Q = [], [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, typ)
        ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        P.append([p.x, p.y, p.z])
        Q.append([q.x, q.y, q.z, q.w])
    return np.array(ts), np.array(P), np.array(Q)


def pose_at(ts, P, Q, t):
    """线性插值 + nlerp（角度很小，与 slerp 差别可忽略）。"""
    i = np.searchsorted(ts, t)
    if i <= 0 or i >= len(ts):
        return None
    a = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
    p = (1 - a) * P[i - 1] + a * P[i]
    q0, q1 = Q[i - 1], Q[i]
    if np.dot(q0, q1) < 0:
        q1 = -q1
    q = (1 - a) * q0 + a * q1
    q /= np.linalg.norm(q)
    T = np.eye(4)
    T[:3, :3] = q2R(*q)
    T[:3, 3] = p
    return T


def pc2xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
    a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    return np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)


def header(path):
    with open(path, "rb") as f:
        txt = b""
        while not txt.endswith(b"DATA binary\n"):
            c = f.read(1)
            if not c:
                break
            txt += c
    lines = txt.decode("ascii", "ignore").splitlines()
    flds = [l.split()[1:] for l in lines if l.startswith("FIELDS")][0]
    n = int([l.split()[1] for l in lines if l.startswith("POINTS")][0])
    return flds, n, len(txt)


def bilinear(img, u, v):
    h, w, _ = img.shape
    x0 = np.floor(u).astype(np.int32)
    y0 = np.floor(v).astype(np.int32)
    fx = (u - x0)[:, None]
    fy = (v - y0)[:, None]
    x0c = np.clip(x0, 0, w - 2)
    y0c = np.clip(y0, 0, h - 2)
    c00 = img[y0c, x0c].astype(np.float32)
    c10 = img[y0c, x0c + 1].astype(np.float32)
    c01 = img[y0c + 1, x0c].astype(np.float32)
    c11 = img[y0c + 1, x0c + 1].astype(np.float32)
    return (c00 * (1 - fx) * (1 - fy) + c10 * fx * (1 - fy)
            + c01 * (1 - fx) * fy + c11 * fx * fy)


def main():
    # ---- 参考点云 ----
    f, n, off = header(REF)
    dt = np.dtype([(c, "<f4") for c in f])
    ref = np.memmap(REF, dtype=dt, mode="r", offset=off, shape=(n,))
    ref_xyz = np.stack([ref["x"], ref["y"], ref["z"]], 1).astype(np.float64)
    v = np.asarray(ref["rgb"]).view(np.uint32)
    ref_rgb = np.stack([(v >> 16) & 255, (v >> 8) & 255, v & 255], 1).astype(np.float32)
    print("参考 %d 点，建 KDTree ..." % n)
    tree = cKDTree(ref_xyz)
    del ref, ref_xyz

    # ---- 位姿 ----
    tl, Pl, Ql = read_odom(FL2, "/aft_mapped_lidar_to_init", False)
    tv, Pv, Qv = read_odom(FL2, "/vio/odom_lidar", True)
    print("LIO %d 条  VIO %d 条" % (len(tl), len(tv)))

    # ---- 选帧：按 LIO 时间 6 等分，每段取 FRAMES_PER_SEG 帧 ----
    t0, t1 = tl[0], tl[-1]
    want = []
    for s in range(SEGS):
        a = t0 + (t1 - t0) * s / SEGS
        b = t0 + (t1 - t0) * (s + 1) / SEGS
        for k in range(FRAMES_PER_SEG):
            tc = a + (b - a) * (k + 0.5) / FRAMES_PER_SEG
            i = int(np.argmin(np.abs(tl - tc)))
            want.append((s, tl[i]))
    want_ts = np.array([w[1] for w in want])

    # ---- 读 body 云 ----
    clouds = {}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=FL2, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/cloud_registered_body"]))
    ctyp = get_message("sensor_msgs/msg/PointCloud2")
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, ctyp)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        j = int(np.argmin(np.abs(want_ts - st)))
        if abs(want_ts[j] - st) < 1e-3:
            clouds[j] = pc2xyz(m)
    print("拿到 %d/%d 帧 body 云" % (len(clouds), len(want)))

    # ---- 读图像（只留需要的；裸图约 1 MB/帧，全留会吃掉 1.3 GB 内存） ----
    target_img_ts = np.array([tc - IMG_OFFSET for _, tc in want])
    imgs, ists = [], []
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=RAW, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/left_camera/image"]))
    ityp = get_message("sensor_msgs/msg/Image")
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, ityp)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if np.min(np.abs(target_img_ts - st)) <= 0.02:
            ists.append(st)
            imgs.append(m)
    ists = np.array(ists)
    print("图像 %d 帧" % len(ists))

    K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]])
    m1, m2 = cv2.initUndistortRectifyMap(
        K, DIST, None, K, (640, 512), cv2.CV_32FC1)
    T_lc = np.eye(4)
    T_lc[:3, :3] = RCL
    T_lc[:3, 3] = PCL
    T_lc = np.linalg.inv(T_lc)

    print("\n段  帧               n    MAE(0,0)  最优(dx,dy)  MAE最优  旋转(°/s)")
    shifts = [(dx, dy) for dy in range(-SHIFT, SHIFT + 1) for dx in range(-SHIFT, SHIFT + 1)]
    res = []
    for j, (s, tc) in enumerate(want):
        if j not in clouds:
            continue
        ti = int(np.argmin(np.abs(ists - (tc - IMG_OFFSET))))
        img = cv2.imdecode(np.frombuffer(imgs[ti].data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raw = np.frombuffer(imgs[ti].data, np.uint8).reshape(
                imgs[ti].height, imgs[ti].step)[:, :imgs[ti].width * 3].reshape(
                imgs[ti].height, imgs[ti].width, 3)
            img = raw
        rect = cv2.remap(img, m1, m2, cv2.INTER_LINEAR)
        rect_rgb = rect[:, :, ::-1].astype(np.float32)  # BGR -> RGB

        Twl_c = pose_at(tl, Pl, Ql, tc)
        # 节点口径：image.header = t_img - offset，t_img ≈ t_cloud（实测差 0.4~5 ms）。
        # 相机位姿必须取 t_img = header + offset，绝不能取 t_cloud + offset（差 0.1 s 旋转）。
        t_img = ists[ti] + IMG_OFFSET
        Twl_i = pose_at(tv, Pv, Qv, t_img)
        if Twl_c is None or Twl_i is None:
            continue
        Tcw = np.linalg.inv(Twl_i @ T_lc)
        pw = clouds[j] @ Twl_c[:3, :3].T + Twl_c[:3, 3]
        pc = pw @ Tcw[:3, :3].T + Tcw[:3, 3]
        z = pc[:, 2]
        front = z > 0.3
        u = FX * pc[:, 0] / np.where(front, z, 1) + CX
        v_ = FY * pc[:, 1] / np.where(front, z, 1) + CY
        inframe = front & (u >= 0) & (u < 639) & (v_ >= 0) & (v_ < 511)
        sel = np.where(inframe)[0]
        if len(sel) < 500:
            print("%d  %10.3f  点不足" % (s, tc))
            continue
        if len(sel) > NPT:
            sel = np.random.default_rng(j).choice(sel, NPT, replace=False)
        uu, vv, ww = u[sel], v_[sel], pw[sel]
        d, jr = tree.query(ww, k=1, workers=-1)
        ok = d < 0.02
        uu, vv, refc, dd = uu[ok], vv[ok], ref_rgb[jr[ok]], d[ok]
        if len(uu) < 300:
            continue

        best, bestm = (0, 0), 1e9
        for dx, dy in shifts:
            c = bilinear(rect_rgb, uu + dx, vv + dy)
            e = float(np.abs(c - refc).mean())
            if e < bestm:
                bestm, best = e, (dx, dy)
        c0 = bilinear(rect_rgb, uu, vv)
        err = np.abs(c0 - refc)
        m0 = float(err.mean())
        # 与比色评测同口径（d<5mm 同位点）的子集 MAE，用于和流水线 10.6 对齐
        m5 = float(err[dd < 0.005].mean()) if np.any(dd < 0.005) else float("nan")
        ok5 = float((dd < 0.005).mean())

        # 该帧角速度
        k = int(np.argmin(np.abs(tl - tc)))
        w = 0.0
        if 0 < k < len(tl) - 1:
            dtk = tl[k + 1] - tl[k - 1]
            qa, qb = Ql[k - 1], Ql[k + 1]
            ang = 2 * np.arccos(np.clip(abs(np.dot(qa, qb)), -1, 1))
            w = np.degrees(ang) / dtk
        print("%d  %10.3f  %6d  %8.2f  (%+3d,%+3d)  %8.2f  %6.1f   m5=%6.2f(%.0f%%)" % (
            s, tc - t0, len(uu), m0, best[0], best[1], bestm, w, m5, 100.0 * ok5))
        res.append((s, m0, bestm, best[0], best[1], w, m5))

    if res:
        a = np.array([r[1:] for r in res], dtype=float)
        print("\n汇总: MAE(0,0) 均值 %.2f  MAE(最优位移) 均值 %.2f  平均收益 %.2f" % (
            a[:, 0].mean(), a[:, 1].mean(), (a[:, 0] - a[:, 1]).mean()))
        print("最优位移 dx: 均值 %+.1f  std %.1f   dy: 均值 %+.1f  std %.1f" % (
            a[:, 2].mean(), a[:, 2].std(), a[:, 3].mean(), a[:, 3].std()))
        print("|dx|>2 的帧占比 %.0f%%  |dy|>2 的帧占比 %.0f%%" % (
            100.0 * (np.abs(a[:, 2]) > 2).mean(), 100.0 * (np.abs(a[:, 3]) > 2).mean()))
        print("同口径(d<5mm) MAE 均值 %.2f（流水线单帧 10.63 / 中位 9.94）" % a[:, 5].mean())
        print("dx vs w 相关 %.3f  dy vs w 相关 %.3f" % (
            np.corrcoef(a[:, 2], a[:, 4])[0, 1], np.corrcoef(a[:, 3], a[:, 4])[0, 1]))
    print("DONE_SHIFT_SEARCH")


if __name__ == "__main__":
    main()
