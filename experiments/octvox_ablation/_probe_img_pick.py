#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""探针：官方染色到底用哪一帧图像、哪个时刻的相机位姿。"""
import sys
import numpy as np

sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
sys.path.insert(0, "/home/ywj/git/Super-LIVO/experiments/octvox_ablation")
import singleframe_ab as sf  # noqa: E402

REC = "/home/ywj/slam_data/official_vio_20261114/bag"


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


vio, lio, body, colored = load()
print("vio %d lio %d body %d colored %d" % (len(vio), len(lio), len(body), len(colored)))
imgs = sf.RawImages(body[0][0], body[-1][0])
print("imgs %d, dt img median %.4f" % (len(imgs.t), np.median(np.diff(imgs.t))))
print("dt vio median %.4f, dt body median %.4f"
      % (np.median(np.diff([v[0] for v in vio])), np.median(np.diff([b[0] for b in body]))))
print("\n前 12 帧: body_ts  nearest_img_hdr  d=hdr-ts  lio_ts-ts  vio_ts-ts")
for k in range(12):
    ts = body[k][0]
    j = imgs.nearest(ts)
    lio_ts = lio[k][0]
    vio_ts = vio[k][0]
    print("%.4f  %.4f  %+.4f  %+.4f  %+.4f"
          % (ts, imgs.t[j], imgs.t[j] - ts, lio_ts - ts, vio_ts - ts))
print("\n云帧 ts 相对图像(header+0.1) 的位置统计:")
timg = np.array(imgs.t) + sf.IMG_TIME_OFFSET
bts = np.array([b[0] for b in body])
pos = np.searchsorted(timg, bts)
print("  body_ts - timg[pos-1] 中位 %+.4f"
      % np.median(bts - timg[np.clip(pos - 1, 0, len(timg) - 1)]))
print("  body_ts - timg[pos]   中位 %+.4f"
      % np.median(bts - timg[np.clip(pos, 0, len(timg) - 1)]))
print("  最近图像 hdr 与 body_ts 的差: 中位 %+.4f, p90 |d| %.4f"
      % (np.median([imgs.t[imgs.nearest(t)] - t for t in bts]),
         np.percentile([abs(imgs.t[imgs.nearest(t)] - t) for t in bts], 90)))
print("  body_ts 与 vio_ts 差: 中位 %+.2e" % np.median([vio[k][0] - body[k][0] for k in range(len(vio))]))
print("  body_ts 与 lio_ts 差: 中位 %+.2e" % np.median([lio[k][0] - body[k][0] for k in range(len(lio))]))
