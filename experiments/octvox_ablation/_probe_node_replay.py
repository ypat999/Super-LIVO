#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线复现「A 节点在线语义」与「官方语义」，定位节点产出（fl2_A）为何比离线阶梯差。

节点在线语义（visual_a_node.cpp colorizeFrame）：
    p_w   = T_odom(t_cloud) · p_body
    p_c   = [T_odom(t_img) · T_lc]^-1 · p_w,   t_img = 图像header + 0.1
    图像按 |header - t_cloud| 最近挑选  -> header ≈ t_cloud（比官方晚 0.1 s）
    odom  = /aft_mapped_lidar_to_init（LIO 位姿，非 VIO 修正）

官方语义（LIVMapper.cpp publish_frame_world）：
    图像 header ≈ t_cloud - 0.1（header+0.1 = LIO 时刻），相机位姿用 VIO 的 T_f_w(t_cloud)

本脚本对四种组合分别按 2cm 体素算与 R5（官方染色）的色差，逐级隔离因素。
"""
import sys
import numpy as np

sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
import singleframe_ab as sf  # noqa: E402

REC = "/home/ywj/slam_data/official_vio_20261114/bag"
REF = "/home/ywj/slam_data/official_vio_20261114/ladder/R5.pcd"
VS = 0.02

# (名字, 图像挑选平移, 相机位姿来源, 相机位姿时刻)
#   shift: 0 -> header≈t_cloud（节点选图）；-0.1 -> header≈t_cloud-0.1（官方选图）
#   cam_time: "off" -> 图像header+0.1（节点）；"ts" -> 点云戳（官方）
# 点云世界变换一律用 LIO（与官方 publish_frame_world 相同），只变相机侧。
VARIANTS = [
    ("OFFI    图像h≈ts-.1 + 相机VIO(ts)  ", -0.1, "vio", "ts"),
    ("OFFI-L  图像h≈ts-.1 + 相机LIO(ts)  ", -0.1, "lio", "ts"),
    ("NODE-A  图像h≈ts    + 相机LIO(h+0.1)", 0.0, "lio", "off"),
    ("NODE-B  图像h≈ts    + 相机VIO(h+0.1)", 0.0, "vio", "off"),
]


def load_rec(path):
    r = sf._reader(path)
    tt = {x.name: x.type for x in r.get_all_topics_and_types()}
    vio, lio, body = [], [], []
    while r.has_next():
        tup = r.read_next()
        topic = tup[0]
        if topic not in ("/vio/odom_lidar", "/aft_mapped_lidar_to_init",
                         "/cloud_registered_body"):
            continue
        m = sf.deserialize_message(sf._payload(tup), sf._cls(tt[topic]))
        ts = sf._hdr(m)
        if topic == "/cloud_registered_body":
            xyz, _, _ = sf.cloud_parts(m)
            if len(xyz):
                body.append((ts, xyz))
            continue
        p, q = m.pose.pose.position, m.pose.pose.orientation
        (vio if topic == "/vio/odom_lidar" else lio).append(
            (ts, (q.x, q.y, q.z, q.w), (p.x, p.y, p.z)))
    body.sort(key=lambda a: a[0])
    return vio, lio, body


def mk_track(rows):
    rows = sorted(rows, key=lambda a: a[0])
    return sf.Track([a[0] for a in rows], [a[1] for a in rows],
                    [a[2] for a in rows], sf.GAP_NODE)


def key2(xyz):
    k = np.floor(xyz.astype(np.float64) / VS).astype(np.int64)
    off = 1 << 20
    return ((k[:, 0] + off) << 42) | ((k[:, 1] + off) << 21) | (k[:, 2] + off)


def read_pcd(path):
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
    dt = np.dtype([(c, "<f4") for c in flds])
    arr = np.memmap(path, dtype=dt, mode="r", offset=len(txt), shape=(n,))
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float32)
    u32 = np.asarray(arr["rgb"]).view(np.uint32)
    return xyz, np.stack([(u32 >> 16) & 255, (u32 >> 8) & 255, u32 & 255], 1).astype(np.uint8)


def voxel_mean(keys, cols):
    uk, inv = np.unique(keys, return_inverse=True)
    acc = np.empty((len(uk), 3), np.float64)
    for c in range(3):
        acc[:, c] = np.bincount(inv, weights=cols[:, c].astype(np.float64), minlength=len(uk))
    cnt = np.bincount(inv, minlength=len(uk))
    acc /= np.maximum(cnt, 1)[:, None]
    return uk, acc


def main():
    print("[1/3] 读参考", REF)
    rx, rc = read_pcd(REF)
    rk, racc = voxel_mean(key2(rx), rc)
    del rx, rc
    print("   参考体素 %d" % len(rk))

    print("[2/3] 读包")
    vio_rows, lio_rows, body = load_rec(REC)
    print("   VIO %d / LIO %d / body 云 %d" % (len(vio_rows), len(lio_rows), len(body)))
    trk_lio = mk_track(lio_rows)
    trk_vio = mk_track(vio_rows)
    imgs = sf.RawImages(body[0][0], body[-1][0])
    print("   原图 %d 张" % len(imgs.t))

    print("[3/3] 逐变体投影着色")
    results = {}
    for name, shift, cam_src, cam_time in VARIANTS:
        keys_l, cols_l = [], []
        npts = ncol = nframes = 0
        for ts, p_body in body:
            j = imgs.nearest(ts + shift)
            hdr = imgs.t[j]
            t_cam = hdr + sf.IMG_TIME_OFFSET
            if abs(t_cam - ts) > 0.15:
                continue
            trk_cam = trk_lio if cam_src == "lio" else trk_vio
            T_cam, ok1 = trk_cam.at(ts if cam_time == "ts" else t_cam)
            T_cloud, ok2 = trk_lio.at(ts)
            if not (ok1 and ok2):
                continue
            p_w = sf.xf(T_cloud, p_body)
            p_l = sf.xf(np.linalg.inv(T_cam), p_w)
            p_c = p_l @ sf.RCL.T + sf.PCL[None, :]
            u, v, okz = sf.pinhole(p_c)
            in_fov = sf.fov_custom(u, v)
            zb = sf.zbuffer(u, v, p_c[:, 2], in_fov & okz)
            vis, edge = sf.vis_edge(zb, u, v, p_c[:, 2])
            z = np.where(okz, p_c[:, 2], 1e9)
            margin = np.minimum(np.minimum(u, sf.CW - u), np.minimum(v, sf.CH - v))
            quality = np.clip(margin / 20.0, 0.0, 1.0) * np.minimum(1.0, 1.0 / (1.0 + z / 50.0))
            m = in_fov & okz & vis & ~edge & (quality >= 0.2)

            _, im = imgs.get(j)
            rimg = sf.rectify_u8(im)
            rgb = np.zeros((len(p_body), 3), np.uint8)
            rgb[m] = np.clip(np.rint(sf.bilinear(rimg, u[m], v[m])), 0, 255).astype(np.uint8)
            npts += len(p_body)
            ncol += int(m.sum())
            nframes += 1
            keys_l.append(key2(p_w[m]))
            cols_l.append(rgb[m])

        keys = np.concatenate(keys_l)
        cols = np.concatenate(cols_l)
        del keys_l, cols_l
        vk, vacc = voxel_mean(keys, cols)
        del keys, cols
        results[name] = (vk, vacc)
        print("%s 帧 %d, 着色 %.1f%%, 体素 %d"
              % (name, nframes, 100.0 * ncol / max(npts, 1), len(vk)))

    # 统一体素子集后再算色差，避免各变体覆盖范围不同导致不可比
    common = None
    for vk, _ in results.values():
        common = vk if common is None else np.intersect1d(common, vk)
    pos = np.clip(np.searchsorted(rk, common), 0, len(rk) - 1)
    in_ref = rk[pos] == common
    common = common[in_ref]
    print("\n公共可比体素 %d（含参考）\n" % len(common))
    for name, (vk, vacc) in results.items():
        p = np.searchsorted(vk, common)
        p = np.clip(p, 0, len(vk) - 1)
        ok = vk[p] == common
        q = np.searchsorted(rk, common[ok])
        d = np.abs(vacc[p[ok]] - racc[q]).mean(1)
        print("%s 色差 MAE %.2f, p95 %.0f, >30 %.2f%%"
              % (name, d.mean(), np.percentile(d, 95), 100.0 * (d > 30).mean()))


if __name__ == "__main__":
    main()
