#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""探针：确定官方染色用的图像帧与相机位姿时刻（对照官方 /cloud_registered 颜色）。"""
import sys
import numpy as np

sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
import singleframe_ab as sf  # noqa: E402

REC = "/home/ywj/slam_data/official_vio_20261114/bag"
NFRAME = 60


def load():
    r = sf._reader(REC)
    tt = {x.name: x.type for x in r.get_all_topics_and_types()}
    vio, lio, body, colored = [], [], [], []
    while r.has_next():
        tup = r.read_next()
        topic = tup[0]
        m = sf.deserialize_message(sf._payload(tup), sf._cls(tt[topic]))
        ts = sf._hdr(m)
        if topic == "/vio/odom_lidar":
            p, q = m.pose.pose.position, m.pose.pose.orientation
            vio.append((ts, (q.x, q.y, q.z, q.w), (p.x, p.y, p.z)))
        elif topic == "/aft_mapped_lidar_to_init":
            p, q = m.pose.pose.position, m.pose.pose.orientation
            lio.append((ts, (q.x, q.y, q.z, q.w), (p.x, p.y, p.z)))
        elif topic == "/cloud_registered_body":
            xyz, _, _ = sf.cloud_parts(m)
            if len(xyz):
                body.append((ts, xyz))
        elif topic == "/cloud_registered":
            xyz, rgb, _ = sf.cloud_parts(m)
            if len(xyz) and rgb is not None:
                colored.append((ts, xyz, rgb))
    vio.sort(key=lambda a: a[0])
    lio.sort(key=lambda a: a[0])
    body.sort(key=lambda a: a[0])
    return vio, lio, body, colored


def mk(rows):
    return sf.Track([a[0] for a in rows], [a[1] for a in rows],
                    [a[2] for a in rows], sf.GAP_NODE)


vio, lio, body, colored = load()
trk_vio = mk(vio)
trk_lio = mk(lio)
imgs = sf.RawImages(body[0][0], body[-1][0])
print("vio %d lio %d body %d colored %d imgs %d"
      % (len(vio), len(lio), len(body), len(colored), len(imgs.t)))

idx = np.linspace(10, len(body) - 10, NFRAME).astype(int)
acc = {}
cnt = 0
for k in idx:
    ts, p_body = body[k]
    T_lio, ok = trk_lio.at(ts)
    if not ok:
        continue
    cv_xyz, cv_rgb = colored[k][1], colored[k][2]
    for s_img in (-0.1, 0.0, 0.1):
        j = imgs.nearest(ts + s_img)
        hdr = imgs.t[j]
        for tag, t_pose in (("hdr", hdr), ("hdr+.1", hdr + sf.IMG_TIME_OFFSET), ("ts", ts)):
            T_vio, ok1 = trk_vio.at(t_pose)
            if not ok1:
                continue
            p_w = sf.xf(T_lio, p_body)
            p_c = sf.xf(np.linalg.inv(T_vio), p_w) @ sf.RCL.T + sf.PCL[None, :]
            ud, vd, okz = sf.official_uv(p_c)
            m = sf.fov_official(ud, vd) & okz & (np.linalg.norm(p_c, axis=1) > sf.BLIND_RGB)
            if int(m.sum()) != len(cv_rgb):
                continue
            _, im = imgs.get(j)
            got = np.clip(np.rint(sf.bilinear(im, ud[m], vd[m])), 0, 255).astype(np.int16)
            ref = cv_rgb.astype(np.int16)
            if got.shape != ref.shape:
                continue
            key = (s_img, tag)
            a = acc.get(key, [np.zeros(3), 0])
            a[0] += np.abs(got - ref).sum(0)
            a[1] += len(ref)
            acc[key] = a
    cnt += 1

print("\n参与帧数 %d" % cnt)
print("%8s %8s %10s" % ("img_shift", "pose_t", "MAE"))
for (s_img, tag), (s, n) in sorted(acc.items(), key=lambda x: x[1][0].sum() / max(1, x[1][1])):
    print("%+8.2f %8s %10.2f" % (s_img, tag, (s / max(1, n)).mean()))
