#!/bin/bash
source /opt/ros/humble/setup.bash
python3 - <<'PY'
import sys
sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
import numpy as np
from lio_color_factorial import load_source

for src in ["sl", "e1", "fl2"]:
    frames, odom = load_source(src)
    ts = np.array([f[0] for f in frames])
    op = np.array([o[1] for o in odom])
    print(f"{src}: frames={len(frames)} t=[{ts.min():.2f},{ts.max():.2f}] "
          f"span={ts.max()-ts.min():.1f}s")
    print(f"   odom n={len(odom)} pos范围 x[{op[:,0].min():.1f},{op[:,0].max():.1f}] "
          f"y[{op[:,1].min():.1f},{op[:,1].max():.1f}] z[{op[:,2].min():.1f},{op[:,2].max():.1f}]")
    print(f"   路径长={np.linalg.norm(np.diff(op,axis=0),axis=1).sum():.1f}m  "
          f"首末距={np.linalg.norm(op[-1]-op[0]):.1f}m")
    # 点云与自身传感器位置对齐检验：取第 2 帧
    t, P, pk = frames[2]
    # 插值该帧传感器位置
    st = np.array([o[0] for o in odom]); sp = op
    j = np.searchsorted(st, t); j = min(max(j, 1), len(st) - 1)
    a = (t - st[j - 1]) / (st[j] - st[j - 1]); pos = sp[j - 1] + a * (sp[j] - sp[j - 1])
    d = np.linalg.norm(P - pos, axis=1)
    print(f"   帧2 点到传感器距离: 中位 {np.median(d):.2f}m p10 {np.percentile(d,10):.2f} "
          f"p90 {np.percentile(d[0:60],10) if False else np.percentile(d[d<60],90) if (d<60).any() else -1:.2f}m"
          f" 点数={len(P)} rgb非零={(pk!=0).mean()*100:.0f}%")
PY
