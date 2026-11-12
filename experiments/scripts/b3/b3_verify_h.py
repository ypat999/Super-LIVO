#!/usr/bin/env python3
"""离线核对 B3 / B2 两条残差的解析雅可比（ground truth = 中心差分）。
   B3: e(bg) = Log(A * Exp(bg*dt - Phi)),  A = R_meas_rel^T,  R_meas_rel = cur^T prev
       候选: dt*J_l(e)^-1 * A * J_l(psi)   (新) vs dt*J_r(e)*J_r(psi) (旧) vs dt*J_r(e)*J_l(psi)
   B2: e(Rk) = Log(A * Rk^T * Rp),  右扰机体系 Rk -> Rk*Exp(dth)
       候选: -J_l(e)*R_pred_rel^T (代码现用) vs -J_l(e)^-1 * A (严格推导)
"""
import numpy as np
src = open('/tmp/b3_fdsign.py').read().split('rng = np.random.default_rng')[0]
exec(src)
rng = np.random.default_rng(11)

def rel(X, H):
    return float(np.linalg.norm(X - H) / np.linalg.norm(H))

print("### B3: H = d e / d bg   (期望 = dt * 正单位阵量级)")
worst = {}
for t in range(8):
    dt = 1.0 + 0.25 * t
    bg = np.array([-0.0074, 0.0031, -0.0062])
    Phi = rng.normal(0, 0.02, 3) + np.array([0.03, -0.06, 0.15]) * dt   # 未补偿积分(含真实转动)
    psi = bg * dt - Phi
    R_meas_rel = exp_so3(-psi + rng.normal(0, 0.002, 3))                 # 视觉=真值(小噪声)
    A = R_meas_rel.T
    res = lambda b: so3_log(orthonormal(A @ exp_so3(b * dt - Phi)))
    e0 = res(bg)
    h = 1e-3
    Hfd = np.zeros((3, 3))
    for j in range(3):
        d = np.zeros(3); d[j] = h
        Hfd[:, j] = (res(bg + d) - res(bg - d)) / (2 * h)
    cands = {
        "new dt*Jl(e)^-1*A*Jl(psi)": dt * np.linalg.inv(left_jac(e0)) @ A @ left_jac(psi),
        "old dt*Jr(e)*Jr(psi)":      dt * right_jac(e0) @ right_jac(psi),
        "mid dt*Jr(e)*Jl(psi)":      dt * right_jac(e0) @ left_jac(psi),
        "dt*I":                      dt * np.eye(3),
    }
    s = f"[{t}] dt={dt:.2f} |psi|={np.degrees(np.linalg.norm(psi))*57.3:5.1f}mdeg " \
        f"|e|={np.linalg.norm(e0)*1e3:6.2f}mrad Hfd对角=[{Hfd[0,0]:.3f},{Hfd[1,1]:.3f},{Hfd[2,2]:.3f}] | "
    for k, v in cands.items():
        worst[k] = max(worst.get(k, 0.0), rel(v, Hfd))
        s += f"{k.split()[0]}={rel(v, Hfd):.2e} "
    print(s)
print("最坏:", {k: f"{v:.2e}" for k, v in worst.items()})

print("\n### B2: H = d e / d dth   (代码现用 -J_l(e)*R_pred_rel^T)")
worst2 = {}
for t in range(6):
    Rk = exp_so3(rng.normal(0, 0.3, 3))
    Rp = exp_so3(rng.normal(0, 0.3, 3))
    R_pred_rel = orthonormal(Rk.T @ Rp)
    R_meas_rel = orthonormal(R_pred_rel @ exp_so3(rng.normal(0, 0.004, 3)))
    A = R_meas_rel.T
    res = lambda d: so3_log(orthonormal(A @ orthonormal((Rk @ exp_so3(d)).T @ Rp)))
    e0 = res(np.zeros(3))
    h = 1e-5
    Hfd = np.zeros((3, 3))
    for j in range(3):
        d = np.zeros(3); d[j] = h
        Hfd[:, j] = (res(d) - res(-d)) / (2 * h)
    cands = {"-Jl(e)*Rp_rel^T": -left_jac(e0) @ R_pred_rel.T,
             "-Jl(e)^-1*A": -np.linalg.inv(left_jac(e0)) @ A,
             "-I": -np.eye(3)}
    s = f"[{t}] |e|={np.linalg.norm(e0)*1e3:6.2f}mrad "
    for k, v in cands.items():
        worst2[k] = max(worst2.get(k, 0.0), rel(v, Hfd))
        s += f"{k}={rel(v, Hfd):.2e}  "
    print(s)
print("最坏:", {k: f"{v:.2e}" for k, v in worst2.items()})
