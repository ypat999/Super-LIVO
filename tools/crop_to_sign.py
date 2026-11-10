#!/usr/bin/env python3
"""把地图按招牌 bbox 裁出来（与既有 .section.pcd 同一把尺子），供 qc_validate_v1 对比。"""
import sys
import numpy as np
sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
from qc_metric_screen import read_pcd

REF = ["/home/ywj/slam_data/pcd/retail_street_livo_rgb_v2.section.pcd",
       "/home/ywj/slam_data/pcd/super_livo_visual_a_colored.section.pcd"]


def write_pcd(path, xyz, rgb):
    n = len(xyz)
    pk = ((rgb[:, 0].astype(np.uint32) << 16) | (rgb[:, 1].astype(np.uint32) << 8)
          | rgb[:, 2].astype(np.uint32))
    body = np.empty((n, 16), np.uint8)
    body[:, :12] = np.stack([xyz[:, i].astype(np.float32).view(np.uint8).reshape(n, 4)
                             for i in range(3)], 1).reshape(n, 12)
    body[:, 12:] = pk.view(np.uint8).reshape(n, 4)
    hdr = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
           "FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n"
           f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n")
    with open(path, "wb") as f:
        f.write(hdr.encode() + body.tobytes())


def main(src, dst):
    rs = [read_pcd(p) for p in REF]
    allp = np.concatenate([r[0] for r in rs])
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
    P, C = read_pcd(src)
    vv = P - c0
    u, w, dn = vv @ a1, vv @ a2, vv @ n0
    m = ((u >= ulo) & (u <= uhi) & (w >= wlo) & (w <= whi)
         & (dn >= dlo - 0.05) & (dn <= dhi + 0.05))
    write_pcd(dst, P[m], C[m])
    print(f"{src.rsplit('/',2)[-2]}: 全图 {len(P):,} -> 招牌 bbox {m.sum():,} 点  存 {dst}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
