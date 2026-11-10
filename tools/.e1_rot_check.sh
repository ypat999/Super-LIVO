#!/bin/bash
source /opt/ros/humble/setup.bash
python3 - <<'PY'
import sys
import numpy as np
sys.path.insert(0, "/home/ywj/git/Super-LIVO/tools")
from blackframe_voxel_replay import load_bag
from scipy.spatial.transform import Rotation, Slerp

f_s, o_s = load_bag("/home/ywj/slam_data/singleframe/sl_frames")
f_e, o_e = load_bag("/home/ywj/slam_data/e1_append/e1_frames")
st_s = np.array([o[0] for o in o_s]); p_s = np.array([o[1] for o in o_s])
q_s = np.array([o[2] for o in o_s])
st_e = np.array([o[0] for o in o_e]); p_e = np.array([o[1] for o in o_e])
q_e = np.array([o[2] for o in o_e])
slp_s = Slerp(st_s, Rotation.from_quat(q_s))
slp_e = Slerp(st_e, Rotation.from_quat(q_e))

idx = np.linspace(0, len(f_e) - 1, 40).astype(int)
dt, dr = [], []
for i in idx:
    t = f_e[i][0]
    if t < max(st_s[0], st_e[0]) or t > min(st_s[-1], st_e[-1]):
        continue
    j = np.searchsorted(st_s, t); j = min(max(j, 1), len(st_s) - 1)
    a = (t - st_s[j - 1]) / (st_s[j] - st_s[j - 1])
    pe = p_s[j - 1] + a * (p_s[j] - p_s[j - 1])
    Re = slp_s([t])[0]
    Rl = slp_e([t])[0]
    # 位置：用 e1 自身插值
    k = min(max(int(np.searchsorted(st_e, t)), 1), len(st_e) - 1)
    b = (t - st_e[k - 1]) / (st_e[k] - st_e[k - 1])
    p_ei = p_e[k - 1] + b * (p_e[k] - p_e[k - 1])
    dt.append(np.linalg.norm(p_ei - pe))
    ang = (Rl.inv() * Re).magnitude()
    dr.append(np.rad2deg(ang))
dt, dr = np.array(dt), np.array(dr)
print(f"e1 vs sl: 位置差 均值 {dt.mean()*100:.1f}cm p90 {np.percentile(dt,90)*100:.1f}cm")
print(f"e1 vs sl: 姿态差 均值 {dr.mean():.2f}° p90 {np.percentile(dr,90):.2f}° 最大 {dr.max():.2f}°")

# 换算到与合成级相同的 pairwise 量
pw_t = np.abs(dt[:, None] - dt[None, :]).mean()  # 标量近似（低估）
pw_r = np.abs(dr[:, None] - dr[None, :]).mean()
print(f"pairwise(标量近似) 位置 {pw_t*100:.1f}cm 姿态 {pw_r:.2f}°")
# 等效杠杆：1° 在 8m 深度 ≈ 14cm 横向位移
print(f"旋转折算横向(8m杠杆): {pw_r*np.deg2rad(1)*800:.1f}cm")
