#!/usr/bin/env python3
"""∂/∂ψ Log(A·Exp(ψ)) 的唯一正确闭式：用数值中心差分当 ground truth 反解。"""
import numpy as np
src = open('/tmp/b3_fdsign.py').read().split('rng = np.random.default_rng')[0]
exec(src)

rng = np.random.default_rng(3)
def vee(m):
    return np.array([m[2, 1], m[0, 2], m[1, 0]])

def rel(X, H):
    return float(np.linalg.norm(X - H) / np.linalg.norm(H))

print("== A=I 时: 数值 δ/δψ Log(Exp(ψ)) ==")
psi = rng.normal(0, 0.15, 3)
Hnum = np.zeros((3, 3)); h = 1e-7
for j in range(3):
    d = np.zeros(3); d[j] = h
    Hnum[:, j] = (so3_log(exp_so3(psi + d)) - so3_log(exp_so3(psi - d))) / (2 * h)
for name, X in {"J_l(ψ)": left_jac(psi), "J_r(ψ)": right_jac(psi),
                "J_l(ψ)^-1": np.linalg.inv(left_jac(psi)),
                "J_r(ψ)^-1": np.linalg.inv(right_jac(psi))}.items():
    print(f"   {name:12s} rel={rel(X, Hnum):.3e}")

print("\n== 一般 A: H = ∂/∂ψ Log(A·Exp(ψ)) ==")
worst = {}
for t in range(6):
    phiA = rng.normal(0, 0.12, 3) * np.array([1, 1, 1.5])
    A = exp_so3(phiA)
    psi = rng.normal(0, 0.12, 3)
    e = so3_log(orthonormal(A @ exp_so3(psi)))
    Hn = np.zeros((3, 3))
    for j in range(3):
        d = np.zeros(3); d[j] = h
        Hn[:, j] = ((so3_log(orthonormal(A @ exp_so3(psi + d))) -
                     so3_log(orthonormal(A @ exp_so3(psi - d)))) / (2 * h))
    Jl_e_inv = np.linalg.inv(left_jac(e))
    cands = {
        "J_r(e)J_r(ψ)": right_jac(e) @ right_jac(psi),
        "J_r(e)J_l(ψ)": right_jac(e) @ left_jac(psi),
        "Jl(e)^-1 Jl(ψ)": Jl_e_inv @ left_jac(psi),
        "Jl(e)^-1 Jr(ψ)": Jl_e_inv @ right_jac(psi),
        "Jl(e)^-1 AdA Jl(ψ)": Jl_e_inv @ (A @ left_jac(psi)),
        "Jl(e)^-1 AdA Jr(ψ)": Jl_e_inv @ (A @ right_jac(psi)),
        "Jr(e)^-1 AdA Jl(ψ)": np.linalg.inv(right_jac(e)) @ (A @ left_jac(psi)),
    }
    line = f"[{t}] |φ_A|={np.linalg.norm(phiA)*1e3:6.1f}mrad |ψ|={np.linalg.norm(psi)*1e3:6.1f}mrad |e|={np.linalg.norm(e)*1e3:6.1f}mrad | "
    for k, X in cands.items():
        r = rel(X, Hn)
        worst[k] = max(worst.get(k, 0.0), r)
        line += f"{k}={r:.2e}  "
    print(line)
print("\n最坏相对误差:")
for k, v in sorted(worst.items(), key=lambda x: x[1]):
    print(f"   {k:22s} {v:.3e}")
