#!/usr/bin/env python3
# 官方 FAST-LIVO2 轨迹 vs Super-LIO 轨迹：量化差异与抖动
# 用法: traj_compare.py OFFICIAL.txt SLIO.txt
import sys
import numpy as np

def load(p):
    a = np.loadtxt(p)
    a = a[np.argsort(a[:, 0])]
    return a

def quat2rot(q):
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    r = np.zeros((len(q), 3, 3))
    r[:, 0, 0] = 1 - 2 * (y * y + z * z)
    r[:, 0, 1] = 2 * (x * y - z * w)
    r[:, 0, 2] = 2 * (x * z + y * w)
    r[:, 1, 0] = 2 * (x * y + z * w)
    r[:, 1, 1] = 1 - 2 * (x * x + z * z)
    r[:, 1, 2] = 2 * (y * z - x * w)
    r[:, 2, 0] = 2 * (x * z - y * w)
    r[:, 2, 1] = 2 * (y * z + x * w)
    r[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return r

def umeyama_se3(src, dst):
    # 求 R,t 使 dst ~ R src + t（不估尺度）
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    C = D.T @ S / len(S)
    U, sv, Vt = np.linalg.svd(C)
    d = np.sign(np.linalg.det(U @ Vt))
    Sm = np.diag([1, 1, d])
    R = U @ Sm @ Vt
    t = mu_d - R @ mu_s
    return R, t

def rot_ang_deg(Rs):
    tr = np.trace(Rs.reshape(-1, 3, 3), axis1=1, axis2=2)
    return np.degrees(np.arccos(np.clip((tr - 1) / 2, -1, 1)))

def interp_poses(a, ts):
    """在时刻 ts 上插值轨迹 a（列: t x y z qx qy qz qw）的位置；姿态取最近邻。"""
    t = a[:, 0]
    idx = np.clip(np.searchsorted(t, ts), 1, len(t) - 1)
    lo, hi = idx - 1, idx
    w = (ts - t[lo]) / (t[hi] - t[lo])
    p = a[lo, 1:4] * (1 - w)[:, None] + a[hi, 1:4] * w[:, None]
    q = a[hi, 4:8]  # 最近邻姿态（10Hz 下误差可忽略）
    return p, q, t[hi] - ts

off = load(sys.argv[1])
sli = load(sys.argv[2])

# 公共时间窗（官方比 Super-LIO 早 ~0.4s 启动，取交集）
t0 = max(off[0, 0], sli[0, 0]) + 0.5
t1 = min(off[-1, 0], sli[-1, 0]) - 0.5
m = (sli[:, 0] >= t0) & (sli[:, 0] <= t1)
s = sli[m]
print('公共窗口 %.1f s, Super-LIO %d 帧' % (t1 - t0, len(s)))

# 先在 Super-LIO 时刻上插官方，再成对做 Umeyama 对齐
p_o, q_o, dt_nn = interp_poses(off, s[:, 0])

# 把 Super-LIO 对齐到官方系（SE3 Umeyama，整段刚性对齐 = 两者都当作从同一起点出发）
R_al, t_al = umeyama_se3(s[:, 1:4], p_o)
s_p = s[:, 1:4] @ R_al.T + t_al
# 注意: p_off ~ R_al p_s + t_al  =>  R_off ~ R_al R_s
Rs_s = R_al @ quat2rot(s[:, 4:8])
Rs_o = quat2rot(off[:, 4:8])
R_o = quat2rot(q_o)

dp = p_o - s_p
dist = np.linalg.norm(dp, axis=1)
drel = np.einsum('nij,njk->nik', R_o.transpose(0, 2, 1), Rs_s)  # R_o^T R_s
ang = rot_ang_deg(drel)

print('\n== 绝对差（SE3 对齐后，官方为参考）==')
print('位置差 |dp|: 均值 %.3f m 中位 %.3f  P90 %.3f  最大 %.3f' %
      (dist.mean(), np.median(dist), np.percentile(dist, 90), dist.max()))
print('分轴 dp: x 均值 %+.3f std %.3f | y %+.3f std %.3f | z %+.3f std %.3f' %
      (dp[:, 0].mean(), dp[:, 0].std(), dp[:, 1].mean(), dp[:, 1].std(),
       dp[:, 2].mean(), dp[:, 2].std()))
print('旋转差: 均值 %.2f° 中位 %.2f° P90 %.2f° 最大 %.2f°' %
      (ang.mean(), np.median(ang), np.percentile(ang, 90), ang.max()))

# 姿态差分解为 roll/pitch/yaw（用官方姿态系）
eul_o = np.degrees(np.arctan2(R_o[:, 1, 0], R_o[:, 0, 0]))  # yaw
eul_s = np.degrees(np.arctan2(Rs_s[:, 1, 0], Rs_s[:, 0, 0]))
yaw_err = np.angle(np.exp(1j * np.radians(eul_s - eul_o)), deg=True)
pitch_o = np.degrees(np.arcsin(-np.clip(R_o[:, 2, 0], -1, 1)))
pitch_s = np.degrees(np.arcsin(-np.clip(Rs_s[:, 2, 0], -1, 1)))
roll_o = np.degrees(np.arctan2(R_o[:, 2, 1], R_o[:, 2, 2]))
roll_s = np.degrees(np.arctan2(Rs_s[:, 2, 1], Rs_s[:, 2, 2]))
print('yaw 差: 均值 %+.2f° std %.2f° | pitch 差 std %.2f° | roll 差 std %.2f°' %
      (yaw_err.mean(), yaw_err.std(), (pitch_s - pitch_o).std(), (roll_s - roll_o).std()))

# 像素敏感度换算：横向平移误差与旋转误差在深度 Z 处的像素量
fx = 646.78
for Z in (10.0, 20.0):
    px_t = dist.mean() / Z * fx
    px_r = np.tan(np.radians(ang.mean())) * fx
    print('深度 %.0fm: 平移差≈%.1f px, 旋转差≈%.1f px（合计≈%.1f px）' %
          (Z, px_t, px_r, np.hypot(px_t, px_r)))

print('\n== 抖动/平滑度（各自轨迹二阶差分，与对齐无关）==')
for name, a in (('官方', off), ('Super-LIO', sli)):
    a2 = a[(a[:, 0] >= t0) & (a[:, 0] <= t1)]
    v = np.diff(a2[:, 1:4], axis=0) / np.diff(a2[:, 0])[:, None]
    acc = np.diff(v, axis=0) / np.diff(a2[:, 0][1:])[:, None]
    speed = np.linalg.norm(v, axis=1)
    amag = np.linalg.norm(acc, axis=1)
    jerk = np.diff(acc, axis=0)
    print('%-9s 速度 均值 %.2f m/s | 加速度 中位 %.2f P95 %.2f 最大 %.2f m/s² | 加加速度 P95 %.1f' %
          (name, speed.mean(), np.median(amag), np.percentile(amag, 95), amag.max(),
           np.percentile(np.linalg.norm(jerk, axis=1), 95)))

print('\n== 相对位姿差 RPE(1s)（对彩色跨帧一致性最相关的量）==')
step = 10  # 10Hz -> 1s
rel_o_p = np.linalg.norm(off[step:, 1:4] - off[:-step, 1:4], axis=1)
# 在 s 上算 RPE 平移，并与官方同窗 RPE 比较
seg = np.linalg.norm(s_p[step:] - s_p[:-step], axis=1)
seg_o = np.linalg.norm(p_o[step:] - p_o[:-step], axis=1)
rpe = np.abs(seg - seg_o)
print('1s 位移差: 均值 %.3f m 中位 %.3f P90 %.3f' % (rpe.mean(), np.median(rpe), np.percentile(rpe, 90)))

# 帧间旋转差（着色相邻两帧 t_img 差 0.1s，位姿差直接决定跨帧错位）
ang1 = rot_ang_deg(np.einsum('nij,njk->nik', Rs_s[:-1].transpose(0, 2, 1), Rs_s[1:]))
ang1o = rot_ang_deg(np.einsum('nij,njk->nik', R_o[:-1].transpose(0, 2, 1), R_o[1:]))
print('相邻帧旋转: Super-LIO 中位 %.2f° | 官方 中位 %.2f°' % (np.median(ang1), np.median(ang1o)))

print('\n== 首尾闭合 ==')
print('官方 末帧位置 %s' % np.round(off[-1, 1:4], 2))
print('Super-LIO(对齐后) 末帧 %s 官方同刻 %s' %
      (np.round(s_p[-1], 2), np.round(p_o[-1], 2)))
