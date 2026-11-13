#!/usr/bin/env python3
"""用外部参考位姿（官方 FAST-LIVO2）重建阶段一输入集 inputs.npz。

只替换「世界位姿」，其余完全不变：
  - 同一批去畸变 body 云（replay3 的 /lio/cloud_body_dense，逐帧同一份）
  - 同一 ROS 层抽稀 filter_rate=3
  - 同一 0.5 m 最近中心降采样（VoxelGridClosest）
  - 同一时间窗、同一帧集合

用途：检验阶段一「A 的优势」是否来自「A 的图用 A 自己的轨迹建、又用同一轨迹评价」
这一自证结构。注意 body 云的去畸变仍来自 Super-LIVO 的 IMU 传播链，本脚本只换位姿。
"""
import importlib.util
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("builder", os.path.join(HERE, "01_build_maps.py"))
B = importlib.util.module_from_spec(spec)
spec.loader.exec_module(B)

OUT = os.environ.get("OCTVOX_OUT", "/home/ywj/slam_data/octvox_refpose_20261110")
TOL = 1e-2          # 位姿配对容差 (s)：帧周期约 96 ms，10 ms 远小于半周期
REF_TAG = os.environ.get("REF_TAG", "ref1")


def read_ref():
    """复用 09 的时基恢复（FAST-LIVO2 的 odom header 是 wall clock）。"""
    spec9 = importlib.util.spec_from_file_location("cmp09", os.path.join(HERE, "09_compare_external.py"))
    C9 = importlib.util.module_from_spec(spec9)
    spec9.loader.exec_module(C9)
    t, p, q, diag = C9.load_ref(REF_TAG)
    print("ref diag:", json.dumps(diag, ensure_ascii=False))
    return t, p, q


def quat_xyzw_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    os.makedirs(OUT, exist_ok=True)
    rt, rp, rq = read_ref()
    print(f"ref poses={len(rt)} span={rt[0]:.3f}..{rt[-1]:.3f}")

    bag = B.read_bag(B.BAG, ["/lio/cloud_body_dense"])
    dense = bag["/lio/cloud_body_dense"]
    sel = [(t, m) for t, m in dense if B.WINDOW[0] <= t <= B.WINDOW[1]]
    print(f"window dense frames={len(sel)}")

    pts_list, stamps, raw_n, ds_n, dts = [], [], [], [], []
    dropped = 0
    for t, msg in sel:
        j = int(np.searchsorted(rt, t))
        cand = [k for k in (j - 1, j, j + 1) if 0 <= k < len(rt)]
        k = min(cand, key=lambda x: abs(rt[x] - t))
        dt = abs(rt[k] - t)
        # FAST-LIVO2 对约 7% 的帧不发布位姿。丢弃这些帧而不是就近顶替：
        # 顶替会引入 ~1 帧周期甚至更大的位姿误差，比缺帧更糟。
        if dt > TOL:
            dropped += 1
            continue
        body = B.pc2_xyz(msg)
        body = body[np.isfinite(body).all(axis=1)]
        sub = body[::B.FILTER_RATE]
        ds = B.voxel_closest(sub)
        R = quat_xyzw_to_R(rq[k])
        pts_list.append((R @ ds.T).T + rp[k])
        stamps.append(t)
        raw_n.append(len(body))
        ds_n.append(len(ds))
        dts.append(dt)

    arr = np.zeros((len(pts_list), max(ds_n), 3), dtype=np.float32)
    for i, p in enumerate(pts_list):
        arr[i, :len(p)] = p
    np.savez_compressed(os.path.join(OUT, "inputs.npz"), pts=arr,
                        n=np.array(ds_n, dtype=np.int32),
                        stamps=np.array(stamps), raw_n=np.array(raw_n, dtype=np.int32))
    pair = dict(n_frames=len(stamps), dropped_no_pose=dropped,
                pose_source=f"fastlivo2_{REF_TAG}",
                pairing_dt_ms_p50=float(np.percentile(dts, 50) * 1e3),
                pairing_dt_ms_p90=float(np.percentile(dts, 90) * 1e3),
                pairing_dt_ms_max=float(np.max(dts) * 1e3),
                window=list(B.WINDOW))
    with open(os.path.join(OUT, "pose_source.json"), "w") as f:
        json.dump(pair, f, indent=1)
    print(json.dumps(pair, indent=1))


if __name__ == "__main__":
    main()
