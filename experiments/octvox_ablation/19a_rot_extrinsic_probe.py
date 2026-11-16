#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判定"视觉旋转轴与 LIO 不一致"是外参 R_cl 的问题，还是视觉估计本身的问题。

⚠⚠【§36：本脚本的绝对数字作废，只保留"存在轴不一致"这一定性结论】⚠⚠
两处口径错误（均在 §36 用实测钉死）：
 1) 外参侧别：V.R_CL 实测是 **b→c**（v_c = R_CL·v_b），故 body→camera 的共轭应为
    R_CL·L·R_CLᵗ；本脚本拿 M 与 R_clᵀ 比较，侧别反了。
 2) 视觉解用的是**裸 5-point / Procrustes**（RT.rel_pair），§36 实测该模型在
    |t|≈0.06 m、d̄≈10 m 的低视差下退化：同窗口对 IMU 残差 0.351°（其中定偏差 0.199°），
    而深度辅助模型只要 0.048° ⇒ 19a 的 1.293° 主要是**估计器退化**，
    不是外参、也不是时间同步（旧结论"排除外参与时间同步"方向对，但依据不成立）。
⇒ 后续请用 19e_rot_channel_quality.py（新口径重测）+ 19f（重复性）+ 19g（t_c 敏感性）；
   本脚本保留仅作错误链条的历史证据。

若只有 R_cl 错，则旋转**角幅值不变、轴被共轭**。故对大量帧对拟合正交 M:
    r_cam ≈ M · (± r_body)
看残差能否压到 0.1° 量级，并比较 M 与 R_clᵀ。

【§36：本脚本的绝对数字作废，只保留其定性结论】
 1) 本脚本把比较用的外参写成 R_clᵀ·[·]·R_cl（相机←body 的侧别取反）。§36 用重力检验
    + 侧别网格 + 3-DOF 自由拟合三方确认 V.R_CL 实测是 **b→c**，共轭应为 R_CL·[·]·R_CLᵗ，
    统一走 19e 的 b2c()。⇒ 这里 M 与 R_clᵀ 的"接近/不接近"不作数。
 2) 取样从 k=0 起、且未做 Δt_camera 对齐（旧版假定 50 ms 半周期）。§36 实测
    LiDAR 周期≈扫描时长（头尾戳不可分辨），图像内容比其戳晚 **≈110 ms**。
 3) 比较对象是含抖动的 LIO 位姿差，而非 200 Hz IMU 积分。
 ⇒ "轴对不上、拟合 M 后仍 1.293°"这个**量级**不可引用；1.293° 里有相当一部分是
    口径错误本身。有效结论仅一条：残差不是纯角幅值缩放（幅值比 med≈1.06），
    即视觉与 LIO 的差异**不只是尺度问题**。旋转通道质量一律以 19e/19f 为准。
"""
import sys
import numpy as np
sys.path.insert(0, '/home/ywj/git/Super-LIVO/experiments/octvox_ablation')
import _p0_bag as B
import _p0_visual as V
import _p0_rot as RT

NPAIR = int(sys.argv[1]) if len(sys.argv) > 1 else 120
STEP = int(sys.argv[2]) if len(sys.argv) > 2 else 10
BASE = '/home/ywj/slam_data/retail_abl/base.txt'
tl, te, _ = B.read_lidar_stamps(stride=1)
tum = np.loadtxt(BASE)
lio = V.Traj(tum[:, 0], tum[:, 1:4], tum[:, 4:8])
idx = [k for k in range(0, len(tl) - STEP, STEP)][:NPAIR]
idx = [k for k in idx if 0.05 < tl[k + 1] - tl[k] < 0.15]
frames = B.read_images(sorted(set(idx) | set(k + 1 for k in idx)))
gray = {k: V.prep_gray(frames[k][1]) for k in frames}

rc, rb, thv, thl = [], [], [], []
for k in idx:
    d = RT.rel_pair(gray[k], gray[k + 1])
    if d['R'] is None or d['n_inl'] < 60 or d['inlier_ratio'] < 0.5:
        continue
    _, R0, v0 = lio.at([tl[k]])
    _, R1, v1 = lio.at([tl[k + 1]])
    if not (v0[0] and v1[0]):
        continue
    Riel_b = R1[0].T @ R0[0]
    if V.angle_of(Riel_b) < 0.25:          # 旋转太小时轴方向是纯噪声，剔除
        continue
    rc.append(V.log_so3(d['R']))
    rb.append(V.log_so3(Riel_b))
    thv.append(V.angle_of(d['R']))
    thl.append(V.angle_of(Riel_b))
rc = np.array(rc)
rb = np.array(rb)
ratio = np.linalg.norm(rc, axis=1) / np.linalg.norm(rb, axis=1)
print('样本 %d / %d（|r_lio|>0.25°）' % (len(rc), len(idx)))
print('角幅值 visual med=%.3f°  lio med=%.3f°' % (np.median(thv), np.median(thl)))
print('|r_vis|/|r_lio| med=%.3f  p10=%.3f  p90=%.3f'
      % (np.median(ratio), np.percentile(ratio, 10), np.percentile(ratio, 90)))


def fit(P, Q):
    """正交 M 使 Q ≈ M P。"""
    C = Q.T @ P
    U, _, Vt = np.linalg.svd(C)
    return U @ np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))]) @ Vt


print('\n--- A. 不做共轭（等价假设 R_cl=I）---')
for s in (1, -1):
    e = np.degrees(np.linalg.norm(s * rb - rc, axis=1))
    print('  s=%+d  r_cam vs r_body      med=%6.3f° p75=%6.3f° p90=%6.3f°'
          % (s, np.median(e), np.percentile(e, 75), np.percentile(e, 90)))

print('\n--- B. 用当前 R_cl 共轭：r_cam = R_clᵀ r_body ---')
best = None
for s in (1, -1):
    e = np.degrees(np.linalg.norm(s * (V.R_CL.T @ rb.T).T - rc, axis=1))
    print('  s=%+d  med=%6.3f° p75=%6.3f° p90=%6.3f°'
          % (s, np.median(e), np.percentile(e, 75), np.percentile(e, 90)))

print('\n--- C. 拟合正交 M: r_cam = M (s·r_body) ---')
for s in (1, -1):
    M = fit(rb, s * rc)
    pred = (M @ rb.T).T
    e = np.degrees(np.linalg.norm(pred - s * rc, axis=1))
    ang = V.angle_of(M.T)
    print('  s=%+d  med=%6.3f° p75=%6.3f° p90=%6.3f°   ∠(M)=%6.2f°  ∠(M·R_cl)=%6.2f°'
          % (s, np.median(e), np.percentile(e, 75), np.percentile(e, 90), ang,
             V.angle_of(M @ V.R_CL)))
    if s == 1:
        Mp = M
    else:
        Mn = M
print('\n  拟合 M(s=+1) row-major:', np.round(Mp.reshape(-1), 4).tolist())
print('  拟合 M(s=-1) row-major:', np.round(Mn.reshape(-1), 4).tolist())
print('  标定 R_clᵀ       row-major:', np.round(V.R_CL.T.reshape(-1), 4).tolist())
