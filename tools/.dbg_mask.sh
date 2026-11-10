#!/bin/bash
python3 - <<'PY'
import sys, re
sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
import numpy as np
from blackframe_voxel_replay import load_bag, make_interpolator
frames, odom = load_bag("/home/ywj/slam_data/singleframe/sl_frames")
pose_at = make_interpolator(odom)
yaml = open("/home/ywj/git/Super-LIVO/solution_a_lightweight/super_livo_visual_a/config/visual_a_params.yaml").read()
fx = float(re.search(r"cam_fx:\s*([\d.]+)", yaml).group(1))
fy = float(re.search(r"cam_fy:\s*([\d.]+)", yaml).group(1))
cx = float(re.search(r"cam_cx:\s*([\d.]+)", yaml).group(1))
cy = float(re.search(r"cam_cy:\s*([\d.]+)", yaml).group(1))
W = int(re.search(r"cam_width:\s*(\d+)", yaml).group(1))
H = int(re.search(r"cam_height:\s*(\d+)", yaml).group(1))
Rcl = np.array([float(v) for v in re.search(r"Rcl:\s*\[([^\]]+)\]", yaml, re.S).group(1).replace("\n", " ").split(",")]).reshape(3, 3)
Pcl = np.array([float(v) for v in re.search(r"Pcl:\s*\[([^\]]+)\]", yaml).group(1).split(",")])
for fi in (0, 1, 2, 20):
    t, Pb, pk = frames[fi]
    R, tv = pose_at(t)
    Pw = Pb.astype(np.float64) @ R.T + tv
    p = (Pw - tv) @ R @ Rcl.T + Pcl
    z = p[:, 2]
    print(f"帧#{fi}: z p5/p50/p95 = {np.percentile(z,[5,50,95]).round(2)}  z>0.3: {(z>0.3).sum()}/{len(z)}")
    m = z > 0.3
    u = fx*p[m,0]/z[m]+cx; v = fy*p[m,1]/z[m]+cy
    print(f"   u p5/p50/p95 = {np.percentile(u,[5,50,95]).round(1)}  v p5/p50/p95 = {np.percentile(v,[5,50,95]).round(1)}  W,H={W},{H}")
    print(f"   在界内(2..W-2,2..H-2): {((u>=2)&(u<W-2)&(v>=2)&(v<H-2)).sum()}")
PY
