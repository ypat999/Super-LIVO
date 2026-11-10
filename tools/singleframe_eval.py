#!/usr/bin/env python3
"""P0 实验分析：单帧着色质量 vs 跨帧累积质量。

对每个系统分别算：
  单帧级 —— 逐帧彩色点云里落在招牌区域的部分，算栅格椒盐率 + Q1/Q2/Q3；
  地图级 —— 同一系统保存下来的累积地图，同一招牌区域，同一套指标。

判读：
  单帧就已经差  => 投影/时间/遮挡问题（Colorizer 本身）
  单帧干净、地图差 => 跨帧累积（重复观测 + 体素中位色回填）

用法（需先 source /opt/ros/humble/setup.bash）：
  python3 singleframe_eval.py <refA.pcd> <refB.pcd> \
      sl <sl_frames_bag> <sl_map.pcd>  fl2 <fl2_frames_bag> <fl2_map.pcd>
"""
import sys
import numpy as np
from scipy.ndimage import median_filter, distance_transform_edt

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


# ------------------------------------------------- PointCloud2 解码
def pc2_to_xyzrgb(msg):
    n = msg.width * msg.height
    buf = np.frombuffer(msg.data, np.uint8).reshape(n, msg.point_step)
    off = {f.name: (f.offset, f.datatype) for f in msg.fields}

    def f32(name):
        o, _ = off[name]
        return buf[:, o:o + 4].copy().view(np.float32).reshape(n)
    xyz = np.stack([f32("x"), f32("y"), f32("z")], 1).astype(np.float64)
    rgb = None
    if "rgb" in off or "rgba" in off:
        key = "rgb" if "rgb" in off else "rgba"
        o, _ = off[key]
        pk = buf[:, o:o + 4].copy().view(np.uint32).reshape(n)
        rgb = np.stack([(pk >> 16) & 0xFF, (pk >> 8) & 0xFF, pk & 0xFF], 1).astype(np.uint8)
    elif "r" in off:                      # 有些驱动用三个 uint8
        rgb = np.stack([buf[:, off[c][0]] for c in "rgb"], 1).astype(np.uint8)
    return xyz, rgb


def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read_bag(path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    if topics:
        flt = rosbag2_py.TopicsFilter()
        flt.topics = list(topics)
        reader.set_filter(flt)
    names = set(topics) if topics else None
    out = {}
    while reader.has_next():
        item = reader.read_next()
        topic, rest = item[0], list(item[1:])
        payload = next((x for x in rest if isinstance(x, (bytes, bytearray))), None)
        stamp = next((x for x in rest if isinstance(x, (int, float))), None)
        if payload is None:                      # 某些版本返回对象而非 tuple
            payload = getattr(item, "data", None)
            stamp = getattr(item, "time_stamp", stamp)
        if names is not None and topic not in names:
            continue
        out.setdefault(topic, []).append((stamp, payload))
    return out


# ------------------------------------------------- 指标（与 qc_validate_v1 同定义）
def raster_metrics(P, dn, rgb, u, w, px=0.002):
    if rgb is None or len(u) < 3000:
        return None
    iu = np.floor((u - u.min()) / px).astype(np.int64)
    iw = np.floor((w - w.min()) / px).astype(np.int64)
    W_, H_ = int(iu.max()) + 1, int(iw.max()) + 1
    if W_ * H_ > 6_000_000:
        return None
    pix = iw * W_ + iu
    zb = np.full(W_ * H_, np.inf)
    np.minimum.at(zb, pix, dn)
    filled = np.isfinite(zb).reshape(H_, W_)
    if filled.mean() < 0.02:
        return None
    L = np.zeros(W_ * H_)
    order = np.argsort(-dn)
    L[pix[order]] = (0.299 * rgb[order][:, 0] + 0.587 * rgb[order][:, 1]
                     + 0.114 * rgb[order][:, 2])
    L = L.reshape(H_, W_)
    _, near = distance_transform_edt(~filled, return_indices=True)
    L = L[near[0], near[1]]
    med = median_filter(L, 5, mode="nearest")
    return float(np.mean((np.abs(L - med) > 40)[filled])), int(filled.sum())


def q123(P, K1=10, K2=25, r=0.03, minb=6):
    from scipy.spatial import cKDTree
    tree = cKDTree(P)
    dd, ii = tree.query(P, k=K2)
    q1 = float(dd[:, K1 - 1].mean())
    nb3 = (dd < r).sum(1)
    q3 = float(np.mean(nb3 < minb))
    Q = P[ii[:, :K2]] - P[ii[:, :K2]].mean(1, keepdims=True)
    C = np.einsum("nki,nkj->nij", Q, Q) / K2
    ev = np.linalg.eigvalsh(C)[:, 0]
    rg = np.sqrt(np.clip(ev, 0, None))
    return q1, float(np.std(rg) / max(np.mean(rg), 1e-12)), q3


def main(ref_a, ref_b, sys_specs):
    from qc_metric_screen import read_pcd
    PA, RA = read_pcd(ref_a)
    PB, RB = read_pcd(ref_b)
    allp = np.concatenate([PA, PB])
    c0 = allp.mean(0)
    _, _, vt = np.linalg.svd(allp - c0, full_matrices=False)
    n0 = vt[2]
    if n0[2] < 0:
        n0 = -n0
    a1 = np.array([n0[1], -n0[0], 0.0]); a1 /= np.linalg.norm(a1)
    a2 = np.cross(n0, a1)
    v = allp - c0
    ulo, uhi = np.percentile(v @ a1, [0.5, 99.5])
    wlo, whi = np.percentile(v @ a2, [0.5, 99.5])
    dlo, dhi = np.percentile(v @ n0, [0.5, 99.5])
    px = 0.002
    npx = int((uhi - ulo) / px) * int((whi - wlo) / px)
    print(f"招牌 bbox: u {ulo:.2f}..{uhi:.2f}  w {wlo:.2f}..{whi:.2f}  dn {dlo*100:.1f}..{dhi*100:.1f} cm"
          f"   2mm 栅格 {npx:,} 像素")

    for name, frame_bag, map_pcd in sys_specs:
        print(f"\n===== {name} =====")
        recs = read_bag(frame_bag, None)
        ct = [t for t in recs if "colored" in t or "cloud_registered" in t][0]
        odom = next(((t, recs[t]) for t in recs if "odom" in t or "aft_mapped" in t), None)
        o_ts, o_R, o_t = [], [], []
        if odom:
            for ts, d in odom[1]:
                m = deserialize_message(d, Odometry)
                o_ts.append(ts / 1e9)
                o_R.append(quat_R(m.pose.pose.orientation))
                p = m.pose.pose.position
                o_t.append([p.x, p.y, p.z])
        o_ts = np.array(o_ts)

        frames = []
        for ts, d in recs[ct]:
            msg = deserialize_message(d, PointCloud2)
            if msg.width == 0 or not any(f.name == "x" for f in msg.fields):
                continue
            P, C = pc2_to_xyzrgb(msg)
            if C is None:                        # 官方 LIO 帧没有 RGB，跳过
                continue
            if "colored_cloud" in ct:
                k = np.searchsorted(o_ts, ts / 1e9, side="right") - 1
                if k < 0:
                    continue
                P = P @ o_R[k].T + o_t[k]
            vv = P - c0
            u, w, dn = vv @ a1, vv @ a2, vv @ n0
            m = ((u >= ulo) & (u <= uhi) & (w >= wlo) & (w <= whi)
                 & (dn >= dlo - 0.15) & (dn <= dhi + 0.15))
            if m.sum() < 200:
                continue
            frames.append((P[m], dn[m], C[m], u[m], w[m]))
        print(f"  含 RGB 且落在招牌 bbox 的帧: {len(frames)}  "
              f"每帧中位 {int(np.median([len(f[0]) for f in frames])) if frames else 0} 点")

        print(f"   {'累计帧数':<10}{'点数':>9}{'栅格填充%':>11}{'椒盐率%':>10}"
              f"{'Q1 mm':>9}{'Q2':>8}{'Q3 %':>8}")
        for k in (1, 2, 5, 10, 20, 40, len(frames)):
            if k > len(frames) or k < 1:
                continue
            sel = frames[:k]
            P = np.concatenate([f[0] for f in sel])
            dn = np.concatenate([f[1] for f in sel])
            C = np.concatenate([f[2] for f in sel])
            U = np.concatenate([f[3] for f in sel])
            W = np.concatenate([f[4] for f in sel])
            r = raster_metrics(P, dn, C, U, W, px)
            q = q123(P)
            if r is None:
                print(f"   {k:<10}{len(P):>9,}{'填充不足':>11}")
            else:
                print(f"   {k:<10}{len(P):>9,}{r[1]/npx*100:>11.1f}{r[0]*100:>10.2f}"
                      f"{q[0]*1000:>9.2f}{q[1]:>8.3f}{q[2]*100:>8.3f}")
        try:
            PM, CM = read_pcd(map_pcd)
            vv = PM - c0
            u, w, dn = vv @ a1, vv @ a2, vv @ n0
            m = ((u >= ulo) & (u <= uhi) & (w >= wlo) & (w <= whi)
                 & (dn >= dlo - 0.15) & (dn <= dhi + 0.15))
            r = raster_metrics(PM[m], dn[m], CM[m], u[m], w[m], px)
            q = q123(PM[m])
            if r:
                print(f"   {'完整地图':<10}{m.sum():>9,}{r[1]/npx*100:>11.1f}{r[0]*100:>10.2f}"
                      f"{q[0]*1000:>9.2f}{q[1]:>8.3f}{q[2]*100:>8.3f}")
        except Exception as ex:
            print("  地图级失败:", ex)


if __name__ == "__main__":
    a = sys.argv[1:]
    refs = a[0:2]
    specs = []
    i = 2
    while i < len(a):
        specs.append((a[i], a[i + 1], a[i + 2]))
        i += 3
    main(refs[0], refs[1], specs)
