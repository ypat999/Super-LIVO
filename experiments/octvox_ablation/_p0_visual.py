#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Phase 0 视觉前端：相邻帧相对旋转估计（LK 双向跟踪 + 网格化 + RANSAC 5-point）。

约定（务必与代码一致，否则符号全反）：
  世界系 w、LiDAR/body 系 b、相机系 c。
  TUM 轨迹里的四元数是 **b→w**（body 到 world）。

  ⚠️ 侧别为**实测**结论（见 exp_records §36），不是照抄 FAST-LIVO2 的文档措辞：
  本包的 Rcl 实际是 **b→c**： p_c = R_cl · p_b （等价地 R_bc = R_cl）。
  证据：静止段重力均值 ĝ=[-0.008,-0.011,0.992]（单位 g），
        R_cl·ĝ = (-0.005,-1.000,-0.014) → 相机系"上"= −y ✓（本相机 y 轴朝下）
        R_clᵗ·ĝ = ( 1.000, 0.014, 0.005) → 图像右 ✗
  另：6 候选侧别×转置网格、以及 3-DOF 自由正交拟合（起点含 D=I 与随机）
      都独立收敛到同一侧别 ⇒ 不是笔误。Pcl 仍是 body 系的相机原点（杠杆臂）。
  ⇒ Pcl 不随侧别改动而变，但 **R_cl 的转置位置必须逐处核对**。

  findEssentialMat(pts1, pts2) 给出 E = [t]_× R，满足 x2^T E x1 = 0，
  故其 R 把相机 1 的坐标旋到相机 2： R_21^c = R_{w2c,2} · R_{w2c,1}^T 。
  由 R_{w2c} = R_cl · R_{b2w}^T（因 R_wc = R_wb · R_bc = R_wb · R_clᵗ）得
      R_21^c = R_cl · ( R_{b2w,2}^T · R_{b2w,1} ) · R_clᵗ
  即"体相对旋转"左乘 R_cl 的共轭。因为 R_cl 恒定，用相对旋转时 Pcl 不出现。
"""
import numpy as np
import cv2

# ---- 官方标定 /home/ywj/slam_data/Retail_Street/camera_pinhole.yaml -----------
# 原始 1280x1024，scale=0.5 => bag 里直接是 640x512 的图（实测 enc=rgb8）。
K_FULL = np.array([[1293.56944, 0.0, 626.91359],
                   [0.0, 1293.3155, 522.799224],
                   [0.0, 0.0, 1.0]])
DIST = np.array([-0.076160, 0.123001, -0.00113, 0.000251], dtype=np.float64)
W0, H0 = 640, 512                                   # bag 内实际分辨率（实测 shape=(512,640,3)）
# ⚠️ scale=0.5 是 FAST-LIVO2 对原始 1280x1024 的降采样系数；本 bag 的图像
# **已经是** 640x512，配套内参就是 K_FULL/2，不能再降一次。
K_HALF = np.array([[K_FULL[0, 0] * 0.5, 0.0, K_FULL[0, 2] * 0.5],
                   [0.0, K_FULL[1, 1] * 0.5, K_FULL[1, 2] * 0.5],
                   [0.0, 0.0, 1.0]])
IMG_DOWN = 1                     # >1 时在此基础上再降采样（算力实验用）
W, H = int(W0 / IMG_DOWN), int(H0 / IMG_DOWN)
K_USE = K_HALF.copy()
K_USE[:2] /= IMG_DOWN

R_CL = np.array([0.00610193, -0.999863, -0.0154172,
                 -0.00615449, 0.0153796, -0.999863,
                 0.999962, 0.00619598, -0.0060598]).reshape(3, 3)   # b→c（实测，见模块头）
P_CL = np.array([0.0194384, 0.104689, -0.0251952])                  # 相机原点在 body 系

_MAP1 = _MAP2 = None


def prep_gray(rgb):
    """去畸变 + 转灰度 + 可选再降采样。一次建表，逐帧复用。"""
    global _MAP1, _MAP2
    if _MAP1 is None:
        # 源图就是 640x512、内参 K_HALF —— 畸变系数是归一化的，与分辨率无关，
        # 故 K 用 K_HALF 而不是 K_FULL（用 K_FULL 会把画面裁到角落）。
        _MAP1, _MAP2 = cv2.initUndistortRectifyMap(
            K_HALF, DIST, None, K_HALF, (W, H), cv2.CV_16SC2)
    g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    u = cv2.remap(g, _MAP1, _MAP2, cv2.INTER_LINEAR)
    if IMG_DOWN > 1:
        u = cv2.resize(u, None, fx=1.0 / IMG_DOWN, fy=1.0 / IMG_DOWN,
                       interpolation=cv2.INTER_AREA)
    return u


def grid_balance(pts, ncell=4, per_cell=None, shape=(H, W)):
    """把特征点按 ncell×ncell 网格均衡：每格最多保留 per_cell 个（取角点响应最强）。

    goodFeaturesToTrack 的返回顺序即 Harris 响应降序，故每格先到先得即可。
    返回 (pts_bal, cell_of_pt)。
    """
    if len(pts) == 0:
        return pts.astype(np.float32).reshape(-1, 2), np.zeros(0, int)
    p = np.asarray(pts, dtype=np.float32).reshape(-1, 2)
    h, w = shape
    cx = np.clip((p[:, 0] * ncell / w).astype(int), 0, ncell - 1)
    cy = np.clip((p[:, 1] * ncell / h).astype(int), 0, ncell - 1)
    cid = cy * ncell + cx
    if per_cell is None:
        per_cell = max(1, int(np.ceil(len(p) / (ncell * ncell))))
    keep, cnt = [], np.zeros(ncell * ncell, int)
    for i, c in enumerate(cid):
        if cnt[c] < per_cell:
            cnt[c] += 1
            keep.append(i)
    keep = np.array(keep, int)
    return p[keep], cid[keep]


def track_pair(g1, g2, max_pts=400, fb_thresh=1.0, ncell=4, levels=4,
               win=21, iters=30):
    """双向 LK：返回通过前后向一致性检查的对应点（原始帧坐标）。"""
    p0 = cv2.goodFeaturesToTrack(g1, maxCorners=max_pts, qualityLevel=0.01,
                                 minDistance=8, blockSize=7)
    if p0 is None or len(p0) < 20:
        return None, None, 0
    p0 = p0.reshape(-1, 2)
    p0, _ = grid_balance(p0, ncell=ncell)
    p0 = p0.reshape(-1, 1, 2).astype(np.float32)
    if len(p0) < 20:
        return None, None, 0
    pq = dict(winSize=(win, win), maxLevel=levels - 1,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, iters, 0.01))
    p1, st1, _ = cv2.calcOpticalFlowPyrLK(g1, g2, p0, None, **pq)
    if p1 is None:
        return None, None, len(p0)
    p0b, st2, _ = cv2.calcOpticalFlowPyrLK(g2, g1, p1, None, **pq)
    ok = (st1.reshape(-1) == 1) & (st2.reshape(-1) == 1)
    a, b, b2 = p0.reshape(-1, 2)[ok], p1.reshape(-1, 2)[ok], p0b.reshape(-1, 2)[ok]
    fb = np.linalg.norm(b2 - a, axis=1)
    sel = fb < fb_thresh
    a, b = a[sel], b[sel]
    return a, b, len(p0)


def rotm(R):
    """数值正交化（SVD）。"""
    u, _, vt = np.linalg.svd(R)
    d = np.sign(np.linalg.det(u @ vt))
    return u @ np.diag([1, 1, d]) @ vt


def angle_of(R):
    """旋转角（度）。必须先投影回 SO(3)：V.R_CL 的正交精度只有 ~1e-6，
    经 b2c(L)=R_CL·L·R_CLᵗ 后乘积矩阵的 trace 可超出 3（实测 +8.6e-7），
    clip 会把它截成 1 ⇒ 凡是小于 ~0.05° 的角度被系统性误报为 0（§36.9 坑）。"""
    c = (np.trace(rotm(R)) - 1.0) * 0.5
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def rel_rotation(g1, g2, thresh=1.0, want_hom=True):
    """一对去畸变灰度图 -> 相对旋转（相机系，c1->c2）及全部诊断量。

    主通道 5-point；另算 Homography 通道用于对比（纯旋转/低基线场景 5-point 退化）。
    """
    out = dict(n_tracked=0, n_inl=0, inlier_ratio=0.0, coverage=0.0, fov=0.0,
               R5=None, th5=np.nan, tdir=None, dh=None, Rh=None, n_inl_h=0,
               parallax_med=np.nan, cond=np.nan)
    a, b, ntr = track_pair(g1, g2)
    out['n_tracked'] = ntr
    if a is None or len(a) < 30:
        return out
    out['n_pairs'] = len(a)
    E, mask = cv2.findEssentialMat(a, b, K_USE, method=cv2.RANSAC, prob=0.999,
                                   threshold=thresh, mask=None)
    if E is None or mask is None:
        return out
    inl = mask.reshape(-1) == 1
    n_inl = int(inl.sum())
    out['n_inl'] = n_inl
    out['inlier_ratio'] = n_inl / max(1, len(a))
    if n_inl < 20:
        return out
    _, R, t, _ = cv2.recoverPose(E, a, b, K_USE, mask=mask)
    out['R5'] = rotm(R)
    out['tdir'] = t.reshape(3) / max(1e-12, np.linalg.norm(t))
    out['th5'] = angle_of(out['R5'])
    # 空间覆盖率：4x4 网格中"有 >=2 内点"的格子占比
    cells = np.zeros((4, 4), int)
    for x, y in a[inl]:
        cells[min(3, int(y * 4 / g1.shape[0])), min(3, int(x * 4 / g1.shape[1]))] += 1
    out['coverage'] = float((cells >= 2).mean())
    # 公共视场：b 落在图内的比例（已在 LK 内保证）+ 内点在 a 中的凸包面积占比
    pts = a[inl]
    lo, hi = pts.min(0), pts.max(0)
    out['fov'] = float(max(0.0, (hi[0] - lo[0]) * (hi[1] - lo[1]) /
                           (g1.shape[1] * g1.shape[0])))
    # 内点对极残差（归一化坐标下的 x2ᵀ[t]×R x1，乘焦距换算成 px）。
    # 不用 findEssentialMat 返回的 E 直接算：其在像素系/归一化系之间语义不统一，
    # 而 recoverPose 的 R、t 明确作用在归一化射线上，量纲可靠。
    hom_a = np.c_[a[inl], np.ones(n_inl, dtype=np.float64)].T
    hom_b = np.c_[b[inl], np.ones(n_inl, dtype=np.float64)].T
    Kinv = np.linalg.inv(K_USE)
    x1 = Kinv @ hom_a
    x2 = Kinv @ hom_b
    tv = np.asarray(t, dtype=np.float64).reshape(3)
    T = np.array([[0.0, -tv[2], tv[1]], [tv[2], 0.0, -tv[0]], [-tv[1], tv[0], 0.0]])
    res = np.abs(np.einsum('ji,jk,ki->i', x2, T @ R, x1))
    out['e_res_med'] = float(np.median(res) * K_USE[0, 0])      # ≈ px
    if want_hom and len(a) >= 8:
        Hm, mh = cv2.findHomography(a, b, cv2.RANSAC, 3.0)
        if Hm is not None and mh is not None:
            hi_ = mh.reshape(-1) == 1
            out['n_inl_h'] = int(hi_.sum())
            out['Rh'] = None
            if hi_.sum() > 8:
                pred = (Hm @ np.c_[a[hi_], np.ones(int(hi_.sum()))].T).T
                pred = pred[:, :2] / pred[:, 2:3]
                out['dh'] = float(np.median(np.linalg.norm(pred - b[hi_], axis=1)))
    return out


def quat_to_rot(q):
    """(qx,qy,qz,qw) -> 3x3（TUM 顺序，标量在后）。"""
    q = np.asarray(q, float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return rotm(np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]))


def rot_to_quat_w(R):
    """3x3 -> (qx,qy,qz,qw)。"""
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def slerp(R1, R2, a):
    """测地插值：R(a) = R1 · Exp(a · Log(R1^T R2))。"""
    w = log_so3(R1.T @ R2)
    n = np.linalg.norm(w)
    if n < 1e-12:
        return rotm(R1)
    u = w / n
    return rotm(R1 @ exp_so3(u * a * n))


def exp_so3(w):
    """so(3) 向量（弧度）-> SO(3)。"""
    n = np.linalg.norm(w)
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    if n < 1e-8:
        return np.eye(3) + K + 0.5 * K @ K
    return np.eye(3) + np.sin(n) / n * K + (1 - np.cos(n)) / n ** 2 * (K @ K)


class Traj:
    """LIO 位姿连续时间轨迹：平移线性插值 + 旋转 slerp（同 pointcloud_colorizer 口径）。"""

    def __init__(self, t, pos, quat):
        self.t = np.asarray(t, float)
        self.p = np.asarray(pos, float)
        self.R = np.stack([quat_to_rot(q) for q in quat])
        self.gap = np.diff(self.t) > 0.3

    def at(self, tq):
        tq = np.atleast_1d(np.asarray(tq, float))
        out_p, out_R, valid = np.zeros((len(tq), 3)), [], np.zeros(len(tq), bool)
        for i, t in enumerate(tq):
            j = np.searchsorted(self.t, t)
            if j <= 0 or j >= len(self.t):
                continue
            lo, hi = j - 1, j
            if self.gap[lo]:
                continue
            a = (t - self.t[lo]) / (self.t[hi] - self.t[lo])
            out_p[i] = self.p[lo] + a * (self.p[hi] - self.p[lo])
            out_R.append(slerp(self.R[lo], self.R[hi], a))
            valid[i] = True
        Rs = np.stack(out_R) if out_R else np.zeros((0, 3, 3))
        full = np.tile(np.eye(3), (len(tq), 1, 1))
        full[valid] = Rs
        return out_p, full, valid


def log_so3(R):
    """SO(3) -> so(3) 向量（弧度）；同样先投影回 SO(3) 再取角/反对称部。"""
    R = rotm(R)
    th = angle_of(R)
    if th < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w / (2 * np.sin(np.radians(th))) * np.radians(th)
