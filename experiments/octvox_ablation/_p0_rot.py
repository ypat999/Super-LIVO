#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉**旋转通道**估计器（P0-C/D 的测量模型）。

为什么不用 findEssentialMat + recoverPose（实测教训，Retail_Street k=10/60/120/300/600/1200）：
  recoverPose 返回的 R 角距中位 ≈179.9°（det=+1、配对 t 下对极残差仅 0.003–1.3 px），
  即它给出的是 4 个可行分解中**另一个同样自洽**的解。
  根因：本数据 100 ms 帧间基线只有厘米级、场景深度 ≥2 m，视差 ≲1°，
  E = [t]×R 中 t 幅值趋零 ⇒ E 秩亏、U/V 在零空间内不确定 ⇒ 分解歧义无法靠 cheirality 消除。
  ⇒ 单目两帧几何**不能**当作高可信 SE(3) 测量（评审第二条的实测确认）。

本模块因此只建 3-DOF 纯旋转模型：
      x2_i  ≈  R · x1_i        （x 为归一化单位视线）
其最小二乘解是正交 Procrustes（Kabsch）闭式解，无歧义、无迭代发散。
平移方向 tdir 仅作参考记录（低视差下同样不可信），**不进优化**。

离群点：先用 findHomography(RANSAC) 剔除外点（近纯旋转时 H≈K R K⁻¹ 仍是合法单应，
不退化），再用纯旋转重投影像素残差迭代重阈值 2 次。

输出 σ_θ（弧度）= σ_px / f ·sqrt(3 / tr(N))，N = Σ_i (I − ŷ_i ŷ_iᵀ) 为旋转信息矩阵，
即理想 isotropic 观测下的先验精度；P0-D 用"实测重复估计散布 / 本 σ_θ"检验其是否被低估。

---------------------------------------------------------------------------
【19b 的合成真值补充结论】纯旋转模型在**有视差**时的偏差 ≈ |t|/d̄ 弧度，
与像素噪声无关（0.1 px 只贡献 0.004°）：|t|=0.03 m / d̄=3 m ⇒ **0.465°**；
0.10/3 ⇒ 1.468°；0.03/12 ⇒ 0.120°。plane 与 diverse 场景几乎相同（Procrustes 不用深度）。
本 bag 实测 |dp| 中位 0.055 m ⇒ 视差偏差 ≈ 0.6°/d̄₅ₘ，与 19a 的 1.293° 同量级，完全解释。
⇒ 旋转通道的测量模型**必须带米制深度**。下面 `estimate_rotation_d` 即该模型：

      x2_i  ~  normalize( R · x1_i  +  t · λ_i ),      λ_i = q · x1_i = 1 / d_i

t 取 **LIO 的相机系帧间平移**（模长与方向都用 LIO，因为第一阶段只修旋转），
q = n̂/d̄ 是"主导平面"的 3 参数逆深度场，与 R（3 参数）共 6 自由度一起解。
它比裸 Procrustes 多用了 5.5 cm 量级的已知基线，因而能把视差耦合的轴偏差压下去；
代价是 λ 的**逐点离面残差**（多平面/植被/动态物）仍会漏进 R —— 这正是 P0-C 要量的东西。
σ_θ 改由 LM 解处的 JᵀJ 边缘协方差给出：|t|/d̄ 越小，R 与 q 越接近退化，
该协方差自动膨胀 ⇒ 可直接当 Σ_vis,k 用，不需要拍脑袋设常数。
"""
import numpy as np
import cv2
from scipy.optimize import least_squares

import _p0_visual as V

K = V.K_USE
KI = np.linalg.inv(K)
F = 0.5 * (K[0, 0] + K[1, 1])


def _bearing(pts):
    """(N,2) 像素 -> (N,3) 单位视线。"""
    h = np.c_[np.asarray(pts, np.float64), np.ones(len(pts))]
    x = (KI @ h.T).T
    return x / np.linalg.norm(x, axis=1, keepdims=True)


def procrustes_rot(x1, x2):
    """最小化 Σ‖x2 − R x1‖² 的 R（正交 Procrustes，det=+1）。"""
    C = x2.T @ x1
    U, _, Vt = np.linalg.svd(C)
    d = np.sign(np.linalg.det(U @ Vt))
    return V.rotm(U @ np.diag([1.0, 1.0, d]) @ Vt)


def _px_resid(a, b, R):
    """纯旋转模型下 a->b 的重投影像素误差（把 ŷ 投回像素系再比 b）。"""
    pred = (K @ (R @ _bearing(a).T)).T
    pred = pred[:, :2] / pred[:, 2:3]
    return np.linalg.norm(pred - np.asarray(b, np.float64), axis=1)


def estimate_rotation(a, b, seed_px=1.0, iters=2, min_inl=20):
    """对应点 (a,b) -> dict(R, th_deg, n_inl, inlier_ratio, px_med, px_p90, sigma_deg, info)。

    seed_px: 首轮用 homography RANSAC 的像素阈值；后续用纯旋转残差重阈值。
    """
    out = dict(R=None, th_deg=np.nan, n_inl=0, inlier_ratio=0.0, px_med=np.nan,
               px_p90=np.nan, sigma_deg=np.nan, coverage=0.0, fov=0.0, tdir=None)
    n = len(a)
    if n < 8:
        return out
    Hm, mh = cv2.findHomography(np.float32(a), np.float32(b), cv2.RANSAC, seed_px)
    if Hm is None or mh is None or mh.reshape(-1).sum() < min_inl:
        m = np.ones(n, bool)
    else:
        m = mh.reshape(-1) == 1
    for it in range(iters + 1):
        R = procrustes_rot(_bearing(a[m]), _bearing(b[m]))
        e = _px_resid(a, b, R)
        keep = e < seed_px
        if keep.sum() < min_inl:
            break
        if it and np.array_equal(keep, m):
            m = keep
            break
        m = keep
    if m.sum() < min_inl:
        return out
    R = procrustes_rot(_bearing(a[m]), _bearing(b[m]))
    e = _px_resid(a[m], b[m], R)
    yhat = R @ _bearing(a[m]).T                      # (3,M)
    N = np.eye(3) * len(e) - yhat @ yhat.T           # 旋转信息矩阵
    sg = 1.4826 * np.median(e) / F                    # robust σ_px -> 弧度
    m_pts = max(1, len(e))
    # 每条单位视线贡献 (I − ŷŷᵀ)（迹 2）⇒ N ≈ (2M/3)I ⇒ Cov = σ²N⁻¹
    # ⇒ 单轴 σ_θ = σ_rad·sqrt(3/(2M))
    out.update(R=V.rotm(R), th_deg=V.angle_of(R), n_inl=int(m.sum()),
               inlier_ratio=float(m.mean()), px_med=float(np.median(e)),
               px_p90=float(np.percentile(e, 90)),
               sigma_deg=float(np.degrees(sg) * np.sqrt(1.5 / m_pts)),
               info=N)
    cells = np.zeros((4, 4), int)
    for x, y in a[m]:
        cells[min(3, int(y * 4 / V.H)), min(3, int(x * 4 / V.W))] += 1
    out['coverage'] = float((cells >= 2).mean())
    pts = a[m]
    lo, hi = pts.min(0), pts.max(0)
    out['fov'] = float(max(0.0, (hi[0] - lo[0]) * (hi[1] - lo[1]) / (V.W * V.H)))
    return out


def rel_pair(g1, g2, max_pts=400, fb_thresh=1.0, ncell=4):
    """一对去畸变灰度图 -> 旋转通道测量 + 前端诊断（跟踪数、E 通道 tdir 参考）。"""
    a, b, ntr = V.track_pair(g1, g2, max_pts=max_pts, fb_thresh=fb_thresh, ncell=ncell)
    d = estimate_rotation(a, b) if a is not None else estimate_rotation([], [])
    d['n_tracked'] = int(ntr)
    d['n_pairs'] = 0 if a is None else int(len(a))
    if a is not None and len(a) >= 20:
        E, mask = cv2.findEssentialMat(a, b, K, method=cv2.RANSAC, prob=0.999,
                                       threshold=1.0)
        if E is not None:
            _, R, t, _ = cv2.recoverPose(E, a, b, K, mask=mask)
            d['tdir'] = np.asarray(t, float).reshape(3)
            d['th_E'] = V.angle_of(V.rotm(R))
        else:
            d['th_E'] = np.nan
    else:
        d['th_E'] = np.nan
    return d


# ============================ 深度辅助（平移已知）旋转估计 ============================

def _skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def cam_rel_from_lio(lio, t1, t2, a_ext=1.0):
    """LIO 连续轨迹 -> 相机 1->2 的 (R_c1c2, t_c 在相机 2 系, |t|)。

    侧别（实测，见 _p0_visual 模块头与 exp_records §36）：V.R_CL 是 **b→c**，
    故 R_bc = R_CL、R_wc = R_wb · R_CLᵗ ， C_w = p_wb + R_wb · P_cl
    R    = R_wc2ᵀ R_wc1 ， t = R_wc2ᵀ (C1 − C2)
    a_ext: 可选的相机基线幅值缩放（P0-E 做敏感性用），仅作用于 t 的模长。
    """
    p1, R1, v1 = lio.at([t1])
    p2, R2, v2 = lio.at([t2])
    if not (v1[0] and v2[0]):
        return None, None, np.nan
    Rw1, Rw2 = R1[0] @ V.R_CL.T, R2[0] @ V.R_CL.T
    C1, C2 = p1[0] + R1[0] @ V.P_CL, p2[0] + R2[0] @ V.P_CL
    Rrel = Rw2.T @ Rw1
    tc = Rw2.T @ (C1 - C2)
    if a_ext != 1.0:
        tc = tc * a_ext
    return Rrel, tc, float(np.linalg.norm(tc))


def plane_lam(X1, X2, t, R):
    """给定 R、t，闭式求逐射线 λ_i = 1/d_i：  x2 × (R x1 + t λ) = 0。"""
    c1 = np.cross(X2, (R @ X1.T).T)
    c2 = np.cross(X2, np.broadcast_to(t, X1.shape))
    den = np.einsum('ij,ij->i', c2, c2)
    num = -np.einsum('ij,ij->i', c1, c2)
    lam = np.full(len(X1), np.nan)
    ok = den > 1e-10
    lam[ok] = num[ok] / den[ok]
    return lam


def fit_plane(X1, lam, iters=4, k=2.5):
    """IRLS 拟合 λ = q·x1（主导平面）；返回 (q, 内点掩码, 稳健 MAD)。"""
    good = np.isfinite(lam)
    q, *_ = np.linalg.lstsq(X1[good], lam[good], rcond=None)
    m = good.copy()
    mad = np.nan
    for _ in range(iters):
        r = lam - X1 @ q
        s = 1.4826 * np.median(np.abs(r[good] - np.median(r[good]))) + 1e-12
        mn = good & (np.abs(r) < k * s)
        if mn.sum() < 8:
            break
        q, *_ = np.linalg.lstsq(X1[mn], lam[mn], rcond=None)
        m, mad, good = mn, s, mn
    return q, m, mad


def _procrustes_lam(X1, X2, t, lam, R):
    """给定 λ 求 R：  R x1 ≈ (u·x2) x2 − λ t ≈ ρ_i x2 − λ_i t （ρ_i 为每点正尺度）。"""
    u = (R @ X1.T).T + np.outer(lam, t)
    rho = np.einsum('ij,ij->i', u, X2)
    tgt = X2 * rho[:, None] - np.outer(lam, t)
    return procrustes_rot(X1, tgt)


def _model(p, Ri, Xi, t):
    """returns (u, ‖u‖, R x1) with R = Exp(δθ)·Ri, λ = q·x1."""
    rx = (V.exp_so3(p[:3]) @ Ri) @ Xi.T                      # (3,N)
    u = rx.T + np.outer(Xi @ p[3:6], t)
    return u, np.linalg.norm(u, axis=1, keepdims=True), rx.T


def _res_jac(p, Ri, Xi, Yi, t):
    """e_i = ŷ_i − x2_i，ŷ = normalize(R x1_i + t (q·x1_i))；解析 Jacobian。

    ∂e/∂δθ = −P [R x1]× ，∂e/∂q = P t x1ᵀ ，P = (I − ŷŷᵀ)/‖u‖
    （用 (I+[δ]×)R x1 = R x1 + δ×R x1 = R x1 − [R x1]× δ）
    """
    u, nu, rx = _model(p, Ri, Xi, t)
    y = u / nu
    P = (np.eye(3)[None, :, :] - y[:, :, None] * y[:, None, :]) / nu[:, :, None]
    S = -np.stack([_skew(r) for r in rx])                    # ∂u/∂δθ
    G = t[None, :, None] * Xi[:, None, :]                    # ∂u/∂q
    JP = np.einsum('kij,kjl->kil', P, np.concatenate([S, G], axis=2))
    return (y - Yi).ravel(), JP.reshape(-1, 6)


def estimate_rotation_d(a, b, t_c, R0=None, seed_px=2.0, min_inl=25,
                        f_scale=3e-4, rounds=10):
    """深度辅助旋转估计：6 参数 (δθ, q) 联合 LM。

    a,b    : 对应像素点 (N,2)
    t_c    : 相机 2 系的帧间平移（米），来自 LIO
    R0     : 初值；None 时用纯旋转 Procrustes
    返回 dict：R, th_deg, q, d_med, lam_mad, t_norm, n_inl, px_med, px_p90,
               sigma_deg[3], sigma_rms/max_deg, cov(3x3), usable, reason
    """
    out = dict(R=None, th_deg=np.nan, q=None, d_med=np.nan, lam_mad=np.nan,
               t_norm=float(np.linalg.norm(t_c)) if t_c is not None else np.nan,
               n_inl=0, inlier_ratio=0.0, px_med=np.nan, px_p90=np.nan,
               sigma_deg=None, sigma_rms_deg=np.nan, sigma_max_deg=np.nan,
               cov=None, coverage=0.0, usable=False, reason='')
    n = len(a) if a is not None else 0
    if n < min_inl:
        out['reason'] = 'too_few_pairs'
        return out
    if t_c is None or not np.all(np.isfinite(t_c)):
        out['reason'] = 'no_t'
        return out
    X1, X2 = _bearing(a), _bearing(b)
    t = np.asarray(t_c, float)

    # --- 外点剔除：单应 RANSAC（单应本身含视差项，比纯旋转更适合当门）---
    Hm, mh = cv2.findHomography(np.float32(a), np.float32(b), cv2.RANSAC, seed_px)
    m = (mh.reshape(-1) == 1) if (Hm is not None and mh is not None) else np.ones(n, bool)
    if m.sum() < min_inl:
        m = np.ones(n, bool)
    if R0 is None:
        R = procrustes_rot(X1[m], X2[m])
    else:
        R = np.asarray(R0, float)

    # --- 交替初值：λ ← 闭式；q ← IRLS 主导平面；R ← 加权 Procrustes ---
    q = np.array([0.0, 0.0, 0.2])                     # 5 m 处的平面初值
    Xm, Ym = X1[m], X2[m]
    for _ in range(rounds):
        lam_all = plane_lam(Xm, Ym, t, R)
        ok = np.isfinite(lam_all) & (lam_all > 5e-4) & (lam_all < 2.0)
        if ok.sum() < min_inl:
            break
        q, _, _ = fit_plane(Xm[ok], lam_all[ok])
        Rn = V.rotm(_procrustes_lam(Xm, Ym, t, Xm @ q, R))
        conv = V.angle_of(Rn.T @ R) < 1e-6
        R = Rn
        if conv:
            break

    # --- 鲁棒 LM（soft_l1 抗动态物/离面点），再在保留点上做线性最小二乘取协方差 ---
    idx = np.where(m)[0]
    Xi, Yi, Ri = X1[idx], X2[idx], V.rotm(R)
    if not np.all(np.isfinite(q)):
        q = np.array([0.0, 0.0, 0.2])
    fun = lambda p: _res_jac(p, Ri, Xi, Yi, t)[0]
    jac = lambda p: _res_jac(p, Ri, Xi, Yi, t)[1]
    try:
        s1 = least_squares(fun, np.r_[np.zeros(3), q], jac=jac, method='trf',
                           loss='soft_l1', f_scale=f_scale, x_scale='jac', max_nfev=200)
    except Exception as exc:                                    # noqa: BLE001
        out['reason'] = 'lm_fail:%s:%s' % (type(exc).__name__, str(exc)[:120])
        return out
    u, nu, _ = _model(s1.x, Ri, Xi, t)
    ei = np.linalg.norm(u / nu - Yi, axis=1)
    keep = ei < max(4.0 * f_scale, 3.0 * 1.4826 * np.median(ei))
    if keep.sum() < min_inl:
        keep = ei <= np.percentile(ei, 70)
    pk = idx[keep]
    Xk, Yk = Xi[keep], Yi[keep]
    s2 = least_squares(lambda p: _res_jac(p, Ri, Xk, Yk, t)[0], s1.x,
                       jac=lambda p: _res_jac(p, Ri, Xk, Yk, t)[1],
                       method='trf', x_scale='jac', max_nfev=200)
    Rf = V.rotm(V.exp_so3(s2.x[:3]) @ Ri)
    qf, J = s2.x[3:6], s2.jac
    dof = max(1, 2 * len(Xk) - 6)              # 每条视线只有 2 个有效约束方向
    C = (2.0 * s2.cost / dof) * np.linalg.pinv(J.T @ J)
    C = 0.5 * (C + C.T)
    Cv = np.linalg.eigvalsh(C[:3, :3])          # 升序；[2] = 最差轴
    u, _, _ = _model(s2.x, Ri, Xk, t)           # u = R x1 + t λ（相机 2 系）
    lamk = Xk @ qf
    pp = (K @ u.T).T
    px = np.linalg.norm(pp[:, :2] / pp[:, 2:3] - np.asarray(b[pk], float), axis=1)
    cells = np.zeros((4, 4), int)
    for x, y in a[pk]:
        cells[min(3, int(y * 4 / V.H)), min(3, int(x * 4 / V.W))] += 1
    sd = np.degrees(np.sqrt(np.maximum(Cv, 0.0)))
    out.update(R=Rf, th_deg=V.angle_of(Rf), q=qf, n_inl=int(keep.sum()),
               inlier_ratio=float(keep.sum() / max(1, n)),
               d_med=float(np.median(1.0 / np.maximum(lamk, 1e-6))),
               lam_mad=float(1.4826 * np.median(np.abs(lamk - np.median(lamk)))),
               px_med=float(np.median(px)), px_p90=float(np.percentile(px, 90)),
               sigma_deg=sd, sigma_rms_deg=float(np.degrees(np.sqrt(max(np.mean(Cv), 0.0)))),
               sigma_max_deg=float(sd[2]), cov=C[:3, :3],
               coverage=float((cells >= 2).mean()), usable=True, reason='')
    if out['t_norm'] < 0.008:
        out['usable'] = False
        out['reason'] = 'baseline_too_short(%.3fm)' % out['t_norm']
    return out


def rel_pair_d(g1, g2, t_c, max_pts=400, fb_thresh=1.0, ncell=4, R0=None):
    """一对去畸变灰度图 + LIO 相机系平移 -> 深度辅助旋转测量。"""
    a, b, ntr = V.track_pair(g1, g2, max_pts=max_pts, fb_thresh=fb_thresh, ncell=ncell)
    if a is None:
        return dict(R=None, usable=False, reason='track_fail', n_tracked=int(ntr))
    d = estimate_rotation_d(a, b, t_c, R0=R0)
    d['n_tracked'] = int(ntr)
    d['n_pairs'] = int(len(a))
    d['a'], d['b'] = a, b
    return d
