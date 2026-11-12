#!/usr/bin/env python3
"""离线复现 visual_eskf_inject.cpp 的 residual()/雅可比，用中心差分核 ∂e/∂δbg 的符号与量级。
纯 numpy，与 C++ 同一套 hat/exp_so3/so3_log/left_jac/right_jac/orthonormal。"""
import numpy as np

def hat(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])

def exp_so3(p):
    th = np.linalg.norm(p)
    K = hat(p)
    if th < 1e-8:
        return np.eye(3) + K + 0.5 * (K @ K)
    return np.eye(3) + np.sin(th) / th * K + (1 - np.cos(th)) / (th * th) * (K @ K)

def so3_log(R):
    """与 Eigen::AngleAxisd(R) 一致：theta in [0,pi], v = axis*theta。"""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = float(np.arccos(c))
    if th < 1e-9:
        return np.zeros(3)
    s = np.sin(th)
    if abs(s) > 1e-6:
        w = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / s
    else:  # theta ~ pi，反对称部分退化，从对称部分取轴
        M = (R + np.eye(3)) / 2.0
        k = int(np.argmax(np.diag(M)))
        v = M[:, k]
        nrm = np.linalg.norm(v)
        w = v / nrm if nrm > 1e-12 else np.array([1.0, 0.0, 0.0])
    return w * th

def left_jac(p):
    th = np.linalg.norm(p)
    K = hat(p)
    if th < 1e-8:
        return np.eye(3) - 0.5 * K
    a = (1 - np.cos(th)) / (th * th)
    b = (th - np.sin(th)) / (th ** 3)
    return np.eye(3) + a * K + b * (K @ K)

def right_jac(p):
    return left_jac(-p)

def orthonormal(R):
    U, _, Vt = np.linalg.svd(R)
    O = U @ Vt
    if np.linalg.det(O) < 0:
        U[:, 2] *= -1.0
        O = U @ Vt
    return O

rng = np.random.default_rng(7)
print("residual(bg) = Log( A * Exp(bg*dt - Phi) ),   A = R_meas_rel^T  (机体系约定同 B2)")
print("候选式: H_rr = dt*J_r(e)*J_r(psi)   H_rl = dt*J_r(e)*J_l(psi)   H_neg = -dt*I")
print("判据: 与中心差分 H_fd 的相对误差 rel = ||H - H_fd|| / ||H_fd||\n")
worst = {"rr": 0.0, "rl": 0.0, "neg": 0.0}
for trial in range(8):
    dt = 1.0 + 0.15 * trial
    bg = np.array([-0.0074, 0.0031, -0.0062])                 # rad/s 量级零偏
    psi_true = rng.normal(0, 0.03, 3) + np.array([0.02, -0.05, 0.12])
    Phi = bg * dt - psi_true                                  # 使 bg*dt - Phi = psi_true
    R_meas_rel = exp_so3(psi_true + rng.normal(0, 0.003, 3))  # 视觉测量
    A = R_meas_rel.T

    def res(b):
        return so3_log(orthonormal(A @ exp_so3(b * dt - Phi)))

    e0 = res(bg)
    h = 1e-3
    H_fd = np.zeros((3, 3))
    for j in range(3):
        d = np.zeros(3); d[j] = h
        H_fd[:, j] = (res(bg + d) - res(bg - d)) / (2.0 * h)
    H_rr = dt * right_jac(e0) @ right_jac(bg * dt - Phi)
    H_rl = dt * right_jac(e0) @ left_jac(bg * dt - Phi)
    rel = lambda X: float(np.linalg.norm(X - H_fd) / np.linalg.norm(H_fd))
    r_rr, r_rl, r_neg = rel(H_rr), rel(H_rl), rel(-dt * np.eye(3))
    worst["rr"] = max(worst["rr"], r_rr); worst["rl"] = max(worst["rl"], r_rl)
    worst["neg"] = max(worst["neg"], r_neg)
    print(f"[{trial}] dt={dt:.2f}s |psi|={np.linalg.norm(psi_true)*1e3:6.1f}mrad "
          f"|e|={np.linalg.norm(e0)*1e3:6.1f}mrad  H_fd对角=[{H_fd[0,0]:.4f},{H_fd[1,1]:.4f},{H_fd[2,2]:.4f}]")
    print(f"      rel(J_r*J_r)={r_rr:.3e}  rel(J_r*J_l)={r_rl:.3e}  rel(-dt*I)={r_neg:.3e}")
print(f"\n最坏值: rr={worst['rr']:.3e}  rl={worst['rl']:.3e}  neg={worst['neg']:.3e}")
print("期望: H_fd 对角 ≈ +dt (>0) ⇒ ∂e/∂δbg 为正号; rl≈1e-9 ⇒ 解析式取 dt*J_r(e)*J_l(ψ);")
print("      rr≈|psi|/2 量级 ⇒ 原写 J_r(psi) 确实错(非对角 ~10%); neg≈2 ⇒ 符号绝非负。")
