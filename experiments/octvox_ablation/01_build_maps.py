#!/usr/bin/env python3
"""阶段一 01：提取 OctVox 地图输入点集，并按 A/B/C/D 四策略重建地图。

严格对齐源码语义（docs/OCTVox_UPDATE_AUDIT.md）：
  - 父体素 0.5 m（floor 分格），8 个子体素 0.25 m，每子体素 1 代表点
  - 合并门槛：|p - stored| <= merge_dist(0.1 m)，且 count < max_points(20)
  - A: 增量均值  B: 首次观测  C: 距子体素中心最近  D: 全量累积
  - 输入点集：dense body 云 → 每 3 点取 1（filter_rate=3）→ VoxelGridClosest(0.5 m)
               → 用同 stamp 的后验 odom 变换到世界系

输出目录：/home/ywj/slam_data/octvox_ablation_20261110
"""
import argparse
import json
import os
import time

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

BAG = "/home/ywj/slam_data/geom_replay_20261110/replay3"
OUT = os.environ.get("OCTVOX_OUT", "/home/ywj/slam_data/octvox_ablation_20261110")
WINDOW = (946685507.0, 946685531.5)

RES_PARENT = 0.5
RES_SUB = 0.25
MERGE_DIST = 0.1
MAX_POINTS = 20
FILTER_RATE = 3
UNINIT = 0


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_bag(path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in out:
            cls = PointCloud2 if topic == "/lio/cloud_body_dense" else Odometry
            out[topic].append((stamp_of(deserialize_message(data, cls)),
                               deserialize_message(data, cls)))
    del reader
    for t in out:
        out[t].sort(key=lambda x: x[0])
    return out


def pc2_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    buf = buf.reshape(n, msg.point_step)
    xyz = np.empty((n, 3), dtype=np.float32)
    for i, k in enumerate("xyz"):
        xyz[:, i] = buf[:, off[k]:off[k] + 4].copy().view("<f4")[:, 0]
    return xyz


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def voxel_closest(pts, leaf=RES_PARENT):
    """VoxelGridClosest 语义：idx = round(p/leaf)，每格保留离格中心最近的真实点。"""
    if len(pts) == 0:
        return pts
    idx = np.round(pts / leaf).astype(np.int64)
    center = idx.astype(np.float64) * leaf
    d2 = ((pts - center) ** 2).sum(axis=1)
    order = np.lexsort((d2, idx[:, 2], idx[:, 1], idx[:, 0]))
    s = idx[order]
    first = np.ones(len(s), dtype=bool)
    first[1:] = np.any(s[1:] != s[:-1], axis=1)
    return pts[order[first]]


def subvoxel_keys(pts):
    """返回 (父体素 key(int64), local_idx(int8), 子体素中心(N,3))。"""
    fine = np.floor(pts / RES_SUB).astype(np.int64)
    parent = fine >> 1
    local = (((fine[:, 2] & 1) << 2) | ((fine[:, 1] & 1) << 1) | (fine[:, 0] & 1))
    origin = parent.astype(np.float64) * RES_PARENT
    half = 0.5 * RES_SUB
    center = origin + np.stack([
        np.where(local & 1, 3.0, 1.0) * half,
        np.where(local & 2, 3.0, 1.0) * half,
        np.where(local & 4, 3.0, 1.0) * half], axis=1)
    # 父体素 key 打包成 int64（每轴 20 位，覆盖 ±262 km @0.5 m）。
    # 留 3 位给 local，使 pkey*8+local 仍不溢出 int64（D/M5 用子体素键）。
    off = 1 << 19
    pkey = ((parent[:, 0] + off) << 40) | ((parent[:, 1] + off) << 20) | (parent[:, 2] + off)
    return pkey, local.astype(np.int8), center


class MapBuilder:
    """A/B/C 共用：容量可增长的 (parent, 8) 槽位表。"""

    def __init__(self, mode, cap=600000):
        self.mode = mode
        self.pts = np.zeros((cap, 8, 3), dtype=np.float64)
        self.cnt = np.zeros((cap, 8), dtype=np.int16)
        self.srcf = np.full((cap, 8), -1, dtype=np.int32)
        self.nrep = np.zeros((cap, 8), dtype=np.int32)
        self.centers = np.zeros((cap, 8, 3), dtype=np.float64)
        self.used = 0
        self.index = {}

    def _grow(self):
        cap = len(self.pts) * 2
        for name in ("pts", "centers"):
            a = getattr(self, name)
            b = np.zeros((cap,) + a.shape[1:], dtype=a.dtype)
            b[:len(a)] = a
            setattr(self, name, b)
        for name in ("cnt", "srcf", "nrep"):
            a = getattr(self, name)
            b = np.zeros((cap,) + a.shape[1:], dtype=a.dtype)
            b[:len(a)] = a
            setattr(self, name, b)

    def add_frame(self, world_pts, frame_idx):
        if len(world_pts) == 0:
            return
        pkey, local, center = subvoxel_keys(world_pts)
        merge2 = MERGE_DIST * MERGE_DIST
        for i in range(len(world_pts)):
            k = int(pkey[i])
            li = int(local[i])
            p = world_pts[i]
            slot = self.index.get(k, -1)
            if slot < 0:
                if self.used >= len(self.pts):
                    self._grow()
                slot = self.used
                self.used += 1
                self.index[k] = slot
                self.pts[slot, li] = p
                self.cnt[slot, li] = 1
                self.srcf[slot, li] = frame_idx
                self.centers[slot, li] = center[i]
                continue
            c = int(self.cnt[slot, li])
            if c == UNINIT:
                self.pts[slot, li] = p
                self.cnt[slot, li] = 1
                self.srcf[slot, li] = frame_idx
                self.centers[slot, li] = center[i]
                continue
            if c >= MAX_POINTS:
                continue
            stored = self.pts[slot, li]
            if float(((p - stored) ** 2).sum()) > merge2:
                continue
            if self.mode == "B":            # 首次观测：不动 XYZ
                self.cnt[slot, li] = c + 1
                continue
            if self.mode == "C":            # 距子体素中心最近
                ctr = self.centers[slot, li]
                if float(((p - ctr) ** 2).sum()) < float(((stored - ctr) ** 2).sum()):
                    self.pts[slot, li] = p
                    self.srcf[slot, li] = frame_idx
                    self.nrep[slot, li] += 1
                self.cnt[slot, li] = c + 1
                continue
            # A：增量均值
            self.pts[slot, li] = (stored * c + p) / (c + 1)
            self.cnt[slot, li] = c + 1

    def export(self):
        n = self.used
        mask = np.zeros((n, 8), dtype=bool)
        valid = self.cnt[:n] > UNINIT
        pts = self.pts[:n][valid]
        cnt = self.cnt[:n][valid].astype(np.int32)
        src = self.srcf[:n][valid]
        nrep = self.nrep[:n][valid]
        ctr = self.centers[:n][valid]
        return dict(points=pts, count=cnt, src_frame=src, n_replace=nrep,
                    d_center=np.linalg.norm(pts - ctr, axis=1),
                    n_parent=int((self.cnt[:n] > UNINIT).any(axis=1).sum()),
                    n_sub=int(valid.sum()))


def build(mode, frames, order=None):
    """order: 帧索引序列（控制实验用）；None = 时间顺序。"""
    t0 = time.perf_counter()
    mb = MapBuilder(mode)
    seq = range(len(frames["pts"])) if order is None else order
    for fi in seq:
        mb.add_frame(frames["pts"][fi][:frames["n"][fi]], int(fi))
    m = mb.export()
    m["build_s"] = time.perf_counter() - t0
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--extract-only", action="store_true")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)

    frames_path = os.path.join(OUT, "inputs.npz")
    if not os.path.exists(frames_path):
        bag = read_bag(BAG, ["/lio/cloud_body_dense", "/lio/odom"])
        dense = bag["/lio/cloud_body_dense"]
        odom = bag["/lio/odom"]
        ot = np.array([t for t, _ in odom])
        print(f"dense={len(dense)} odom={len(odom)}")
        sel = [(t, m) for t, m in dense if WINDOW[0] <= t <= WINDOW[1]]
        print(f"window frames = {len(sel)}")

        pts_list, stamps, raw_n, ds_n = [], [], [], []
        for t, msg in sel:
            j = int(np.searchsorted(ot, t))
            cand = [k for k in (j - 1, j, j + 1) if 0 <= k < len(ot)]
            k = min(cand, key=lambda x: abs(ot[x] - t))
            if abs(ot[k] - t) > 2e-3:
                raise RuntimeError(f"odom 无法配对: {t:.6f} (dt={abs(ot[k]-t)*1e3:.3f} ms)")
            body = pc2_xyz(msg)
            body = body[np.isfinite(body).all(axis=1)]
            sub = body[::FILTER_RATE]                 # ROS 层 filter_rate=3
            ds = voxel_closest(sub)                   # 0.5 m 最近中心
            pose = odom[k][1].pose.pose
            R = quat_to_R(pose.orientation)
            tw = np.array([pose.position.x, pose.position.y, pose.position.z])
            pts_list.append((R @ ds.T).T + tw)
            stamps.append(t)
            raw_n.append(len(body))
            ds_n.append(len(ds))

        arr = np.zeros((len(pts_list), max(ds_n), 3), dtype=np.float32)
        for i, p in enumerate(pts_list):
            arr[i, :len(p)] = p
        np.savez_compressed(frames_path, pts=arr, n=np.array(ds_n, dtype=np.int32),
                            stamps=np.array(stamps), raw_n=np.array(raw_n, dtype=np.int32))
        print(f"saved {frames_path}")
        if args.extract_only:
            return

    d = np.load(frames_path)
    frames = {"pts": d["pts"], "n": d["n"], "stamps": d["stamps"]}
    stats = {}
    for mode in ("A", "B", "C"):
        m = build(mode, frames)
        np.savez_compressed(os.path.join(OUT, f"map_{mode}.npz"),
                            points=m["points"], count=m["count"], src_frame=m["src_frame"],
                            n_replace=m["n_replace"], d_center=m["d_center"])
        stats[mode] = {k: v for k, v in m.items()
                       if k != "points" and not isinstance(v, np.ndarray)}
        stats[mode]["map_points"] = int(len(m["points"]))
        stats[mode]["count_hist"] = np.bincount(m["count"], minlength=MAX_POINTS + 1).tolist()
        stats[mode]["d_center_p50_p90_p99"] = [float(x) for x in
                                               np.percentile(m["d_center"], [50, 90, 99])]
        stats[mode]["replace_frac"] = float((m["n_replace"] > 0).mean())
        print(f"[{mode}] points={len(m['points'])} parents={m['n_parent']} "
              f"t={m['build_s']:.1f}s")

    # D：全量累积
    t0 = time.perf_counter()
    keys_all, pts_all, src_all = [], [], []
    for fi in range(len(frames["pts"])):
        w = frames["pts"][fi][:frames["n"][fi]]
        if len(w) == 0:
            continue
        pkey, local, _ = subvoxel_keys(w)
        keys_all.append(pkey * 8 + local)      # 子体素键
        pts_all.append(w)
        src_all.append(np.full(len(w), fi, dtype=np.int32))
    keys_all = np.concatenate(keys_all)
    pts_all = np.concatenate(pts_all)
    src_all = np.concatenate(src_all)
    _, inv, cnts = np.unique(keys_all, return_inverse=True, return_counts=True)
    np.savez_compressed(os.path.join(OUT, "map_D.npz"), points=pts_all,
                        sub=inv, count=cnts[inv], src_frame=src_all)
    stats["D"] = dict(map_points=int(len(pts_all)), n_sub=int(len(cnts)),
                      n_parent=int(len(np.unique(keys_all // 8))),
                      count_hist=[int(x) for x in np.bincount(np.minimum(cnts, MAX_POINTS),
                                                              minlength=MAX_POINTS + 1)],
                      build_s=time.perf_counter() - t0, max_count=int(cnts.max()),
                      total_sub_obs=int(cnts.sum()))

    with open(os.path.join(OUT, "stats.json"), "w") as f:
        json.dump(stats, f, indent=1)
    print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != "count_hist"}
                      for k, v in stats.items()}, indent=1))


if __name__ == "__main__":
    main()
