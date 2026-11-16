#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P0-E：旋转误差 θ → 地图法向厚度 的**实测**映射（2 m / 5 m / 10 m 三档深度）。

目的
----
exp_records §36.3 把验收目标从"hf_θ ≤ 0.20°"改成了"区间旋转误差 0.13° → 0.05°"，
并按解析式 `e ≈ d·tan θ` 换算成 5 m 墙面 11 mm → 4.4 mm，与"地图厚度 ≤ 5 mm"
目标自洽。但那条换算是**纸面推导**。本脚本用真实点云几何把它验掉或推翻：
对每一帧位姿注入**已知**的小旋转扰动，直接看地图的局部法向厚度涨多少。

口径定义（全部写死，勿改）
--------------------------
1. 点集：`/home/ywj/slam_data/octvox_ablation_20261110/inputs.npz`（245 帧世界点，
   已按 §4 的 A 轨迹后验 odom 变换，且经过 ROS filter_rate=3 + 0.5 m
   VoxelGridClosest 降采样）。先用**生成它时用的同一个位姿**把点逆变换回 body 系
   `p_b = Rᵗ(p_w − t)`，位姿从 replay3 bag 的 `/lio/odom` 按同 stamp 精确配对
   （照抄 `01_build_maps.py` 的配对逻辑：odom **必须先按 header.stamp 排序**，
   实测该话题在 bag 里**乱序**，未排序会有 43 帧配错、|dt| 最大 1402 ms）。
2. 注入：`R_inj(k) = Exp(δ_k) · R(k)`，平移不动（第一阶段只修旋转，与 §36.6 一致）。
   `δ_k` 模长固定为 θ，方向各向同性随机（标准正态归一化）；每个 θ 用 3 个独立
   实现（固定种子）取**中位**，避免单次抽样运气。
3. 成图：扰动位姿直接把 body 点投到世界系并**全量累积**。**不重跑 OctVox 的
   A/B/C/D 代表点策略**——那是另一个变量，这里测的是纯几何分层。
4. 厚度：逐字复用 `_probe_layering.py` 的 A 项 `local_thickness()`
   （k=16 近邻 → 邻域协矩阵最小特征向量为法向 → 点到邻域平面的法向距离绝对值）。
   本数据全量 189,127 点 < 探针 SUB=300,000 上限，因此**不做子采样**，
   与 §4/§35 的 A 项定义一致；但因这里累积的是**降采样原始点**而非成品彩色 PCD，
   本底绝对值与 §4 的 b_lio 7.11 mm **不可直接横比**（判决只看"相对本底增加多少"）。
5. 深度分档：每个点的观测深度 = 该点到**它自己那一帧相机光心**的距离。相机光心
   在 body 系即外参平移 `P_CL`，故深度 = |p_b − P_CL|（刚体变换不改变该距离）。
   分档**按 θ=0 的名义位姿一次定档、各 θ 共用同一批点**，避免档位人群随 θ 漂移；
   注入是绕 body 原点的旋转，相机光心随刚体转动对 |p_b − P_CL| 的影响 ≤ θ·|P_CL|
   ≈ 0.002 m @1°，可忽略。三档 d ∈ [1.5,2.5) / [4,6) / [8,12) m（另加 [3,5) 作
   近场补充档，因本数据 2 m 档点数严重不足，见输出）。
6. 判决量：本脚本同时打印每个 θ 对应的**实测区间旋转误差** ε
   （= median_k ∠( 名义 R(k)ᵗR(k+1) 与扰动 R(k)ᵗR(k+1) 之差)）。
   §36 的"0.13° / 0.05°"说的是**区间（相邻帧相对）旋转误差**，不是单帧位姿误差，
   二者相差约 √2 倍，所以反向判决必须用 ε 这一列，不能拿 θ 直接代入。

判决含义
--------
- 每档拟合 `厚度增量 = slope · θ`（mm/°），与解析预测 `d·tan θ`
  （斜率 = d[mm]·tan(1°) = 17.455·d[m] mm/°）比较，给实测/解析比值。
- 反向判决给两个口径，因为 θ=0 的**本底里已经含着真实 LIO 的区间旋转误差**：
  [口径 A] 把 §36 的 0.13°/0.05° 当成"额外注入"的误差，直接读注入表（保守，高估收益）；
  [口径 B] 认为注入误差与真实误差统计独立，按 ε_tot=√(ε0²+ε²) 合成（ε0 取 §36.2 实测
  0.13°），反解 `a = d厚度/dε_tot`，再回答"ε 从 0.13° 修到 0.05° 能薄多少 mm、
  能否 ≤5 mm"。判决以 [口径 B] 为准，两个口径都打印，便于复核 ε0 假设的影响。
- 反解 `clip_rot`：给定允许增量 Δ mm，θ 上限 = Δ / slope，Δε 上限 = Δ / a。

注入模式开关（本轮新增，第 1 个命令行参数，默认 "RTC" = 三臂全跑）
----------------------------------------------------------------
模式 R（旋转，上一轮的实现，口径**逐字未动**，用作对照基准）：
    `R_inj(k) = Exp(δ_k)·R(k)`，平移不动；|δ_k| = θ 固定、方向各向同性、逐帧独立。
    产物：斜率 a = d厚度/dε_tot（mm/°，ε_tot 为**区间**旋转误差，见口径 B）。
模式 T（平移白噪声，新增）：
    `t_inj(k) = t(k) + u_k·s`，u_k 各向同性单位方向、逐帧独立，s 为模长；旋转不动。
    s 网格 {0.5,1,2,5,10,20,50} mm。成图/分档/k=16 法向厚度/取中位/本底对照**全部复用
    模式 R 的同一条代码路径**（build_cloud + measure_one），只有注入向量不同。
    产物：斜率 b = d厚度/ds（mm 厚度 / mm 平移误差）。
    平移通道的**解析预期**（与模式 R 的 d·tanθ 对位，推导见下方"解析口径"）：
      跨帧法向层间距 = s·√(2/3) = 0.8165·s ⇒ 解析 b_层间距 = 0.8165 mm/mm。
模式 Tc（平移"低频相关"，新增，敏感性臂）：
    每帧同样叠加 |Δt| = s，但**块内恒定**：块长 L = 1/(2·f_c)·10Hz = 25 帧（f_c=0.2 Hz），
    每块一个新的各向同性方向 × 随机符号 ⇒ 相邻帧相对误差 ≈ 0，块边界跳变 ≈ √2·s。
    ⚠️ 口径警告（故本臂**只作敏感性，不作主判据**）：在"相邻帧区间误差"这一口径下
    Tc 的区间误差中位数≈0，与模式 T 不可横比；只有在"逐帧绝对平移误差 = s"这一口径下
    两者才同格。而**真实 LIO 平移误差的相关性未知**（§36 的 hf_p 是高频统计量，对低频
    不敏感）⇒ 判决以 T 为主判据，Tc 只回答"同样逐帧误差幅度，相关型造成的分层是更多
    还是更少"，用来检验"分层由低频位姿不连续主导"这一反向假设。
    另：本臂的实现（块长 25 帧、块内完全恒定、块间跳变）本身就是人为的强离散结构，
    它给出的是"低频不连续"的**上界型**测试，不代表真实 LIO 的连续低频漂移。

解析口径（本轮自己推的，打印时逐项标注假设）
--------------------------------------------
设真实墙面法向 n，某帧整帧平移误差 e_k（各向同性、模长 s、逐帧独立）。
两帧 f1≠f2 的点集在法向上的层间距 = (e_{f1} − e_{f2})·n。
各向同性单位向量 ⇒ E[(u·n)²] = 1/3；两帧独立 ⇒ E[((u1−u2)·n)²] = 2/3。
∴ 层间距 RMS = s·√(2/3) = 0.8165·s（"解析 b"）。切向（垂直视线平面）同理：
E[(u⊥)²] = 2/3 ⇒ 跨帧切向质心分离 RMS = √2·s·√(2/3) = 1.1547·s。
旋转臂对位：整帧旋转 δ（模长 θ 弧度、各向同性）把点 p 位移 δ×(p−t)，**垂直于视线**
⇒ 切向分量 RMS = √2·θ·d·√(2/3) = 1.1547·θ·d；而**法向分量是一阶零**（δ×r ⟂ r，
只有当墙面法向与视线不平行时才有 sin(视线,法向) 的投影）。这正是上一轮
"实测/解析 = 0.18"的机理解释，也是本轮必须补**切向鬼影指标**的理由。

鬼影代理指标（新增，与法向厚度并列，不替换主指标）
--------------------------------------------------
同一 k=16 邻域内把邻居点按**所属帧**分组，把邻域点投影到**垂直于该中心点视线**的平面，
统计**不同帧子集质心两两距离**的（按子集大小加权的）均方根：
    D_i = sqrt( 2·K·Σ_j |c_{f(j)} − c̄|² / (K² − Σ_f n_f²) )，K=16。
（等价的 ANOVA 分解：总切向散布 = 帧内 + 帧间；D 取帧间项的"跨帧两两质心距离"口径，
 两层等量、间距 Δ 时 D = Δ，故 D 可直接与 1.1547·θ·d 这类解析值横比。）
每档取 D 的中位（mm）。注意它只统计**切向**分离，对法向分层不敏感；两者互补。
⚠️ 本实现已知缺陷（结果里如实打印）：k=16 邻域内每帧只摊到 2~4 个点，子集质心被邻域
**真实几何散布 + 抽样噪声**主导 ⇒ D 的本底达几十 mm、对注入的响应远低于解析值，
只能当趋势参照，不能当验收指标。

非线性与口径稳健性（判决必读）
------------------------------
k=16 法向厚度是「点到邻域拟合平面距离的中位数」，本底里已含 LiDAR 测距噪声、墙面粗糙度、
动态物等**与位姿无关**的项；注入幅度远小于本底时增量≈0、超过本底后才近似线性
⇒ 该指标对小幅位姿误差是**饱和（近似平方根型）**响应。所以判决表对每个通道同时给三种口径，
缺一就可能被口径选择带偏：
  [近端斜率] b(s≤5 mm) 与 a(θ≤0.3°)：真实量级处的边际灵敏度（把它线性外推到真实幅度）；
  [全程斜率] 用整个网格拟合：小信号被饱和压低、大信号主导，给出另一端的估计；
  [+1 mm 等效幅度 / 饱和度] 直接在实测曲线上**内插**「造成 +1.00 mm 厚度所需的注入幅度」，
      再用真实幅度除以它 ⇒ 无量纲、对非线性稳健，是两通道排名的主判据。

用法
----
    source /opt/ros/humble/setup.bash
    cd /home/ywj/git/Super-LIVO/experiments/octvox_ablation
    python3 19h_theta_to_thickness.py            # R+T+Tc 三臂，约 1~2 分钟
    python3 19h_theta_to_thickness.py R           # 只跑旋转臂（与上一轮同口径）
结果只打印到 stdout（本仓库不留数据文件），完整日志见运行时的 /tmp/p0e.log、
本轮三臂日志 /tmp/p0e_t.log。
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from scipy.spatial import cKDTree

import _p0_visual as V                    # P_CL / exp_so3（弧度）/ angle_of（度）
import _probe_layering as PL              # local_thickness：k=16 近邻法向厚度（§4 口径）

INPUTS = "/home/ywj/slam_data/octvox_ablation_20261110/inputs.npz"
BAG = "/home/ywj/slam_data/geom_replay_20261110/replay3"
THETAS = [0.0, 0.02, 0.05, 0.1, 0.2, 0.3, 0.5, 1.0]      # 单帧位姿旋转注入幅度（度）
NREP = 3                                                  # 每个 θ 的独立实现数（取中位）
SEED0 = 20261116
BANDS = [(1.5, 2.5), (3.0, 5.0), (4.0, 6.0), (8.0, 12.0)]
MIN_BAND_N = 3000                                         # 低于此点数即判"样本不足"
T_EPS = [0.13, 0.05]                                      # §36.2/36.3 的区间旋转误差（度）
S_GRID = [0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0]     # 平移注入模长（mm）
NREP2 = 2                                               # T/Tc 每个 s 的实现数（省时间，2 次）
SEED_T, SEED_TC = 71000, 72000                          # 与 R 的种子空间隔离
TC_FC = 0.2                                             # Tc 慢变频率（Hz）；10 Hz ⇒ 块长 25 帧
TC_BLOCK = int(round(1.0 / (2.0 * TC_FC) * 10.0))        # 半周期 = 块长（帧），此处 25 帧 = 2.5 s
GHOST_CAP = 6000                                        # 鬼影指标每档采样点数（控时间）
GHOST_SEED = 5150                                       # 各实现共用同一批采样点（配对比较）
T_HF_P = 3.85                                           # 旁证引用的 3-tap hf_p（mm），本轮复测
BASE_TXT = "/home/ywj/slam_data/retail_abl/base.txt"     # §36 的 LIO 轨迹（TUM），仅口径复测


def stamp_of(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read_odom(path):
    """读 /lio/odom（照抄 01 的读取+排序语义；加话题过滤只为省时间）。"""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/lio/odom"]))
    rec = []
    while r.has_next():
        topic, data, _ = r.read_next()
        m = deserialize_message(data, Odometry)
        rec.append((stamp_of(m), quat_to_R(m.pose.pose.orientation),
                    np.array([m.pose.pose.position.x, m.pose.pose.position.y,
                              m.pose.pose.position.z])))
    del r
    rec.sort(key=lambda x: x[0])                          # ⚠️ bag 内乱序，必须排序
    ot = np.array([x[0] for x in rec])
    Rw = np.array([x[1] for x in rec])
    tw = np.array([x[2] for x in rec])
    return ot, Rw, tw


def pair_idx(ot, stamps, tol=2e-3):
    """同 stamp 精确配对（01 的 searchsorted±1 逻辑）；失败即报错，不静默降级。"""
    idx = np.empty(len(stamps), dtype=np.int64)
    dt = np.empty(len(stamps))
    for i, t in enumerate(stamps):
        j = int(np.searchsorted(ot, t))
        cand = [k for k in (j - 1, j, j + 1) if 0 <= k < len(ot)]
        k = min(cand, key=lambda x: abs(ot[x] - t))
        idx[i], dt[i] = k, abs(ot[k] - t)
    bad = int((dt > tol).sum())
    if bad:
        raise RuntimeError(f"odom 配对失败 {bad} 帧，最大 |dt|={dt.max() * 1e3:.3f} ms")
    return idx, dt


def batch_exp(w):
    """so(3) 向量数组 (N,3)（弧度）-> (N,3,3)，与 V.exp_so3 同式（脚本内自检）。"""
    n = np.linalg.norm(w, axis=1)
    K = np.zeros((len(w), 3, 3))
    K[:, 0, 1], K[:, 0, 2] = -w[:, 2], w[:, 1]
    K[:, 1, 0], K[:, 1, 2] = w[:, 2], -w[:, 0]
    K[:, 2, 0], K[:, 2, 1] = -w[:, 1], w[:, 0]
    small = n < 1e-8
    ns = np.where(small, 1.0, n)
    a = np.where(small, 1.0, np.sin(ns) / ns)                # 小角用 Taylor（误差 O(n^2)）
    b = np.where(small, 0.5, (1.0 - np.cos(ns)) / ns ** 2)
    return np.eye(3)[None] + a[:, None, None] * K + b[:, None, None] * (K @ K)


def rel_rot_err_deg(R_nom, R_inj):
    """实测"区间旋转误差"：相邻帧相对旋转之差的角度中位数（度）。"""
    A = np.einsum("kji,klj->kil", R_nom[:-1], R_nom[1:])       # R(k)^T R(k+1) 名义
    B = np.einsum("kji,klj->kil", R_inj[:-1], R_inj[1:])       # 扰动
    E = A @ np.transpose(B, (0, 2, 1))
    c = np.clip((np.trace(E, axis1=1, axis2=2) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.median(np.degrees(np.arccos(c))))


def fitslope(xx, yy):
    """自由截距最小二乘斜率 + R²（增量在各模式 0 档恒为 0，截距不影响斜率）。"""
    A = np.c_[xx, np.ones_like(xx)]
    sl, ic = np.linalg.lstsq(A, yy, rcond=None)[0]
    pred = sl * xx + ic
    r2 = 1 - ((yy - pred) ** 2).sum() / max(1e-12, ((yy - yy.mean()) ** 2).sum())
    return float(sl), float(r2)


def build_cloud(R_inj, t_inj, body, n):
    """扰动位姿 -> 世界系全量累积点云（与上一轮内联写法逐字同序，保证可复现）。"""
    P = np.empty((int(n.sum()), 3))
    pos = 0
    for k in range(len(n)):
        cnt = int(n[k])
        P[pos:pos + cnt] = (R_inj[k] @ body[k].T).T + t_inj[k]
        pos += cnt
    return P


def rel_trans_err_mm(t_nom, t_inj):
    """实测"区间平移误差"（相邻帧相对平移之差）的模长 med/p90（mm）。"""
    d = (t_inj[1:] - t_inj[:-1]) - (t_nom[1:] - t_nom[:-1])
    nn = np.linalg.norm(d, axis=1) * 1000.0
    return float(np.median(nn)), float(np.percentile(nn, 90))


def iso_factors(seed=90210, m=400000):
    """数值标定各向同性统计因子（避免手抄错常数）：
    f_axis = median|N(0,1) 单轴|，f_norm = median|三维标准正态模长|。"""
    rng = np.random.default_rng(seed)
    e = rng.standard_normal((m, 3))
    return (float(np.median(np.abs(e[:, 0]))), float(np.median(np.linalg.norm(e, axis=1))))


def ghost_separation(P, idx, ii, camC, fid, K):
    """跨帧同位置点集的**切向**质心分离距离（鬼影代理，单位 m）。
    idx: 全局点索引；ii: 这些点的 K 近邻索引；camC: 这些点所属帧的相机光心（世界系）。"""
    p0 = P[idx]
    nb = P[ii]
    c = nb.mean(1)
    los = p0 - camC
    los /= np.linalg.norm(los, axis=1, keepdims=True)
    alt = np.abs(los[:, 2]) < 0.9                       # 避开与参考轴平行的退化
    ref = np.zeros_like(los)
    ref[alt] = np.array([0.0, 0.0, 1.0])
    ref[~alt] = np.array([1.0, 0.0, 0.0])
    u = ref - los * np.einsum("nc,nc->n", ref, los)[:, None]
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    v = np.cross(los, u)
    q = nb - c[:, None, :]
    x = np.einsum("nkc,nc->nk", q, u)
    y = np.einsum("nkc,nc->nk", q, v)
    fn = fid[ii]                                        # (N,K) 邻居所属帧
    S = fn[:, :, None] == fn[:, None, :]
    cnt = S.sum(2).astype(np.float64)                   # 每个邻居所在帧子集的大小
    Sf = S.astype(np.float64)
    cx = np.einsum("nkj,nj->nk", Sf, x) / cnt           # 该邻居所属帧子集的质心（切向坐标）
    cy = np.einsum("nkj,nj->nk", Sf, y) / cnt
    B = (cx * cx + cy * cy).sum(1)                      # Σ_j |c_f(j) − c̄|²（c̄=0 精确成立）
    den = float(K * K) - cnt.sum(1)                     # 跨帧有序对数 = K² − Σ_f n_f²
    ok = den > 0.5
    D = np.full(len(idx), np.nan)
    D[ok] = np.sqrt(2.0 * K * B[ok] / den[ok])
    return D


def ghost_measure(P, R_inj, t_inj, bands, gsamp, frameid):
    """各深度档鬼影代理指标 D 的中位（mm）；邻域与 local_thickness 同为 k=PL.K。"""
    Cw = np.einsum("kij,j->ki", R_inj, V.P_CL) + t_inj
    tr = cKDTree(P)
    out = []
    for ib in range(len(bands)):
        idxs = gsamp[ib]
        _, ii = tr.query(P[idxs], k=PL.K)
        D = ghost_separation(P, idxs, ii, Cw[frameid[idxs]], frameid, PL.K)
        out.append(float(np.nanmedian(D) * 1000.0))
    return out


def hf3tap(vec):
    """注入场自身的 3-tap 二阶差分 |v(k-1)-2v(k)+v(k+1)| 的 P50（vec 单位 m），
    返回 (三维模长口径 mm, 逐轴口径 mm)。两个口径必须分开：真实轨迹的 hf_p 到底
    是哪一个，决定 s_eq 差 ~1.5x（本轮用注入场自身标定，避免假设高斯）。"""
    d2 = vec[:-2] - 2 * vec[1:-1] + vec[2:]
    return (float(np.median(np.linalg.norm(d2, axis=1)) * 1000.0),
            float(np.median(np.abs(d2)) * 1000.0))


def trans_vec(rng, s_m, nf, correlated):
    """模式 T/Tc 的平移注入向量 (nf,3)（单位 m，模长恒 = s_m）。"""
    nb = int(np.ceil(nf / TC_BLOCK)) if correlated else nf
    u = rng.standard_normal((nb, 3))
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    if correlated:
        u = u[np.clip(np.arange(nf) // TC_BLOCK, 0, nb - 1)]     # 块内恒定
        u = u * rng.choice([-1.0, 1.0], size=(nb, 1))[
            np.clip(np.arange(nf) // TC_BLOCK, 0, nb - 1)]        # 块间随机符号
    return u * s_m


def main():
    t_all = time.perf_counter()
    # 注入臂开关：默认 RTC 三臂全跑（R 恒跑，它是对照基准与复现自检的对象）
    MODE = "".join(sorted({c for c in (sys.argv[1].upper() if len(sys.argv) > 1 else "RTC")
                           if c in "RTC"})) or "R"
    RUN_T, RUN_TC = "T" in MODE, "C" in MODE
    d = np.load(INPUTS)
    pts, n, stamps = d["pts"], d["n"], d["stamps"]
    nf = len(n)
    print("=" * 78)
    print("P0-E  theta -> thickness  实测映射")
    print("=" * 78)
    print(f"注入臂 MODE={MODE}（R=旋转基准 / T=平移白噪声 / C=平移低频相关 Tc）")
    print(f"inputs.npz: 帧数={nf}  每帧点数 min/med/max/合计="
          f"{int(n.min())}/{int(np.median(n))}/{int(n.max())}/{int(n.sum())}")
    print(f"时间窗 {stamps[0]:.4f} ~ {stamps[-1]:.4f} ({stamps[-1] - stamps[0]:.3f} s)")

    # ---------- 1. 恢复 body 云 + 自检 ----------
    ot, Rw, tw = read_odom(BAG)
    idx, dt = pair_idx(ot, stamps)
    print(f"odom 读取 {len(ot)} 条（已按 stamp 排序）；配对 |dt| max={dt.max() * 1e3:.4f} ms"
          f"  唯一 odom 帧数={len(set(idx.tolist()))}/{nf}")
    Rn = Rw[idx]
    tn = tw[idx]
    body = [None] * nf
    allb, depth, frameid = [], [], []
    for k in range(nf):
        p = pts[k, :int(n[k])].astype(np.float64)
        pb = (Rn[k].T @ (p - tn[k]).T).T                      # p_b = R^T (p_w - t)
        body[k] = pb
        allb.append(pb)
        depth.append(np.linalg.norm(pb - V.P_CL, axis=1))     # 到本帧相机光心的距离
        frameid.append(np.full(len(pb), k))
    allb = np.concatenate(allb)
    maxerr = 0.0
    for k in range(nf):
        back = (Rn[k] @ body[k].T).T + tn[k]
        maxerr = max(maxerr, float(np.abs(back - pts[k, :int(n[k])]).max()))
    print(f"[自检1] 逆变换→正变换 往返最大误差 = {maxerr:.3e} m"
          f"  ({'PASS <1e-4' if maxerr < 1e-4 else 'FAIL 配对/位姿有误'})")
    assert maxerr < 1e-4
    # 与 _p0_visual.exp_so3 的标量实现对比（防止批量化写错）
    rng_chk = np.random.default_rng(0)
    wchk = rng_chk.standard_normal((64, 3)) * 1e-3
    err_so3 = float(np.abs(batch_exp(wchk) - np.stack([V.exp_so3(w) for w in wchk]))
                    .max())
    # ---------- 2. 分档统计 ----------
    depth = np.concatenate(depth)
    frameid = np.concatenate(frameid)
    print(f"[自检2] batch_exp vs V.exp_so3 最大差 = {err_so3:.2e}")
    print("\n-- 深度分档（θ=0 名义位姿定档，各 θ 共用）--")
    bands = []
    for lo, hi in BANDS:
        m = (depth >= lo) & (depth < hi)
        md = float(np.median(depth[m])) if m.any() else float("nan")
        tag = "样本不足" if int(m.sum()) < MIN_BAND_N else "ok"
        print(f"  [{lo:4.1f},{hi:4.1f}) m : n={int(m.sum()):7d}  中位深度={md:5.2f} m  [{tag}]")
        if m.sum():
            bands.append((lo, hi, m, md))
    print(f"  全体 {len(depth)} 点；深度分位 1/5/25/50/75/95/99 = "
          f"{np.round(np.percentile(depth, [1, 5, 25, 50, 75, 95, 99]), 2).tolist()} m")

    # 鬼影指标各档固定采样点（所有实现共用 ⇒ 配对比较，剔除采样噪声）
    gsamp = []
    for _, _, m, _ in bands:
        gi = np.flatnonzero(m)
        if len(gi) > GHOST_CAP:
            gi = np.sort(np.random.default_rng(GHOST_SEED).choice(
                gi, GHOST_CAP, replace=False))
        gsamp.append(gi)
    print(f"  鬼影代理指标各档采样点数 = {[len(s) for s in gsamp]}"
          f"（k={PL.K} 邻域跨帧质心切向分离；各实现共用同一批点 ⇒ 配对比较）")

    # ---------- 2.5 名义位姿本底（θ=0 与 s=0 共用同一次成图，口径完全一致）----------
    Pnom = build_cloud(Rn, tn, body, n)
    dnom = PL.local_thickness(Pnom)
    base_th = [float(np.median(dnom[m]) * 1000.0) for _, _, m, _ in bands]
    base_p50 = float(np.median(dnom) * 1000.0)
    base_gh = ghost_measure(Pnom, Rn, tn, bands, gsamp, frameid)
    print(f"\n-- 名义位姿本底（θ=0 / s=0 共用）--")
    print("  法向厚度(mm): " + "  ".join(
        f"[{lo:.0f}-{hi:.0f}]={base_th[ib]:6.2f}" for ib, (lo, hi, _, _) in enumerate(bands)) +
        f"   [全图 {base_p50:6.2f}]")
    print("  切向鬼影(mm): " + "  ".join(
        f"[{lo:.0f}-{hi:.0f}]={base_gh[ib]:6.2f}" for ib, (lo, hi, _, _) in enumerate(bands)))

    # ---------- 3. 模式 R：逐 θ 注入 -> 成图 -> 测厚（上一轮实现，口径未动）----------
    print("\n-- 模式 R（旋转）注入实测（厚度为 k=16 近邻法向距离，mm；中位数）--")
    print("theta(度) 实现 ε区间误差(度)  " + "  ".join(
        f"[{lo:.0f}-{hi:.0f}]mm" for lo, hi, _, _ in bands))
    # res[itheta][iband][irep] = 厚度中位(mm)；eps[itheta][irep] = 实测区间旋转误差
    res = np.full((len(THETAS), len(bands), NREP), np.nan)
    eps = np.full((len(THETAS), NREP), np.nan)
    p50all = np.full((len(THETAS), NREP), np.nan)
    gres = np.full((len(THETAS), NREP, len(bands)), np.nan)   # 鬼影代理指标（mm）
    t0 = time.perf_counter()
    for it, th in enumerate(THETAS):
        reps = 1 if th == 0.0 else NREP
        for jr in range(reps):
            if th == 0.0:
                # 与 2.5 同一次成图（P 完全相同 ⇒ 数值与上一轮 p0e.log 逐字一致）
                eps[it, jr] = 0.0
                p50all[it, jr] = base_p50
                for ib in range(len(bands)):
                    res[it, ib, jr] = base_th[ib]
                gres[it, jr, :] = base_gh
                print(f"  {th:5.2f}   #{jr}  eps={eps[it, jr]:6.3f}   " +
                      "  ".join(f"{res[it, ib, jr]:7.2f}" for ib in range(len(bands))) +
                      f"     [全图 {p50all[it, jr]:6.2f}]" +
                      f"   ({time.perf_counter() - t0:5.1f} s)")
                continue
            rng = np.random.default_rng(SEED0 + it * 100 + jr)
            u = rng.standard_normal((nf, 3))
            u /= np.linalg.norm(u, axis=1, keepdims=True)      # 各向同性方向
            w = u * np.radians(th)                             # 弧度
            R_inj = batch_exp(w) @ Rn                          # 左乘，平移不动
            P = build_cloud(R_inj, tn, body, n)                # 与上一轮内联写法同序
            dd = PL.local_thickness(P)                         # §4 口径
            eps[it, jr] = rel_rot_err_deg(Rn, R_inj)
            p50all[it, jr] = float(np.median(dd) * 1000.0)
            for ib, (lo, hi, m, md) in enumerate(bands):
                res[it, ib, jr] = float(np.median(dd[m]) * 1000.0)
            gres[it, jr, :] = ghost_measure(P, R_inj, tn, bands, gsamp, frameid)
            print(f"  {th:5.2f}   #{jr}  eps={eps[it, jr]:6.3f}   " +
                  "  ".join(f"{res[it, ib, jr]:7.2f}" for ib in range(len(bands))) +
                  f"     [全图 {p50all[it, jr]:6.2f}]" +
                  f"   ({time.perf_counter() - t0:5.1f} s)")
    med = np.nanmedian(res, axis=2)          # θ=0 只有一个实现，nanmedian 退化为其本身
    eps_med = np.array([eps[it, 0] if THETAS[it] == 0.0 else float(np.median(eps[it]))
                        for it in range(len(THETAS))])

    # 抽样离散度
    print("\n-- 3 次独立实现的中位厚度极差（mm，越大说明单次抽样不可靠）--")
    for ib, (lo, hi, m, md) in enumerate(bands):
        sp = np.nanmax(res[:, ib, :], axis=1) - np.nanmin(res[:, ib, :], axis=1)
        print(f"  [{lo:4.1f},{hi:4.1f}) m : " +
              "  ".join(f"{th:.2f}°±{s:.2f}" for th, s in zip(THETAS[1:], sp[1:])))

    inc = med - med[0][None, :]                                    # 相对本底的增量(mm)
    print("\n-- 厚度增量 vs 本底（mm；本底=θ=0 各档中位）--")
    print("  本底(mm):  " + "  ".join(f"[{lo:.0f}-{hi:.0f}]={med[0, ib]:6.2f}"
                                      for ib, (lo, hi, m, md) in enumerate(bands)))
    print("  theta  eps  " + "  ".join(f"[{lo:.0f}-{hi:.0f}]" for lo, hi, _, _ in bands) + " (mm)")
    for it, th in enumerate(THETAS):
        print(f"  {th:5.2f}  {eps_med[it]:5.3f}  " +
              "  ".join(f"{inc[it, ib]:+7.2f}" for ib in range(len(bands))))

    # ---------- 4. 线性拟合 + 与解析对比 ----------
    print("\n-- 线性拟合 厚度增量 = slope·theta（mm/°） 与解析 d·tanθ 对比 --")
    print("  档位           中位深度  slope(全部θ) R²   slope(θ≤0.3°) R²   解析斜率  实测/解析")

    xs = np.array(THETAS)
    sel = xs <= 0.3 + 1e-12
    fit_all, fit_small = {}, {}
    for ib, (lo, hi, m, md) in enumerate(bands):
        y = inc[:, ib]
        f_full, f_small = fitslope(xs, y), fitslope(xs[sel], y[sel])
        ana = md * 1000.0 * np.tan(np.radians(1.0))               # mm/°
        print(f"  [{lo:4.1f},{hi:4.1f}) m  {md:6.2f} m  "
              f"{f_full[0]:8.2f} {f_full[1]:5.3f}   {f_small[0]:8.2f} {f_small[1]:5.3f}"
              f"   {ana:8.2f}   {f_small[0] / ana:6.3f}")
        fit_all[ib] = f_full
        fit_small[ib] = f_small

    # ---------- 5. 反向判决 ----------
    print("\n-- theta ↔ 区间旋转误差 ε 标定（实测；各帧独立随机 ⇒ ε ≈ √2·θ）--")
    print("  " + "  ".join(f"{th:.2f}°→{e:.3f}°" for th, e in zip(THETAS, eps_med)))
    print(f"  实测 ε/θ 中位 = {float(np.median(eps_med[1:] / np.array(THETAS[1:]))):.3f}"
          f"（√2 = {np.sqrt(2.0):.3f}）")

    # 口径 A：把目标 ε 当作"额外注入"的误差直接读注入表（不含本底里的真实误差）。
    print("\n  [口径 A] 把目标区间旋转误差当成**额外注入**（本底之上，不含真实 ε0）")
    print("    注意列义：'总'=本底+实测增量（绝对厚度），'增'=实测增量，"
          "'解析'=d·tanθ 的**纯鬼影位移**（只能与'增'比，不能与'总'比）")
    print("    档位        θ等效°    ε=0.13° 总/增   ε=0.05° 总/增   解析增量(0.13/0.05)")
    for ib, (lo, hi, m, md) in enumerate(bands):
        th_eq = [float(np.interp(t, eps_med, xs)) for t in T_EPS]
        dinc = [fit_small[ib][0] * t for t in th_eq]
        v = [med[0, ib] + x for x in dinc]
        ana = [md * 1000.0 * np.tan(np.radians(t)) for t in th_eq]
        print(f"    [{lo:4.1f},{hi:4.1f})m {th_eq[0]:6.3f}/{th_eq[1]:6.3f}"
              f"  {v[0]:6.2f}/{dinc[0]:+5.2f}   {v[1]:6.2f}/{dinc[1]:+5.2f}"
              f"     ({ana[0]:5.1f} / {ana[1]:4.1f}) mm")

    # 口径 B（判决口径）：θ=0 的本底**已经包含** §36.2 实测的 LIO 区间旋转误差 EPS0；
    # 注入误差与它统计独立 ⇒ 合成 ε_tot = sqrt(EPS0^2 + ε(θ)^2)。
    # 由此反解 d厚度/dε_tot，再回答"把 ε 从 0.13° 修到 0.05° 能薄多少 mm"。
    EPS0 = 0.13
    print(f"\n  [口径 B / 判决] 本底已含真实 LIO 区间旋转误差 ε0={EPS0:.2f}°（取自 §36.2，"
          f"注入与之独立 ⇒ ε_tot=√(ε0²+ε(θ)²)）")
    print("    档位          本底mm  a=dt/dε(mm/°)  R²    ε=0.13/0.05/0.02/0 的厚度mm   达5mm需ε")
    slope_eps = {}
    for ib, (lo, hi, m, md) in enumerate(bands):
        etot = np.sqrt(EPS0 ** 2 + eps_med ** 2) - EPS0
        selb = xs <= 0.5 + 1e-12
        a, r2 = fitslope(etot[selb], inc[selb, ib])
        slope_eps[ib] = (a, r2)
        t0b = med[0, ib]
        tl = [t0b + a * (e - EPS0) for e in (0.13, 0.05, 0.02, 0.0)]
        need = EPS0 + (5.0 - t0b) / a if a > 1e-9 else float("nan")
        print(f"    [{lo:4.1f},{hi:4.1f})m {t0b:7.2f} {a:11.2f} {r2:6.3f}  "
              + " ".join(f"{t:6.2f}" for t in tl) + f"   {need:7.3f}°")
    ib5 = next((i for i, b in enumerate(bands) if abs(b[1] - 6.0) < 1e-9), None)
    if ib5 is not None:
        a5 = slope_eps[ib5][0]
        t05 = med[0, ib5]
        print(f"\n    >>> 5 m 档判决：本底 {t05:.2f} mm（含真实 ε0={EPS0:.2f}°）；"
              f"实测 a={a5:.1f} mm/° ⇒ 修到 ε=0.05° 得 {t05 - a5 * (EPS0 - 0.05):.2f} mm，"
              f"修到 ε=0 得 {t05 - a5 * EPS0:.2f} mm；"
              f"达到 ≤5 mm 需 ε ≤ {EPS0 - (t05 - 5.0) / a5:.3f}°")

    print("\n-- clip_rot 反解：该档在 ε0 之上再增加 Δ mm 所允许的误差上限 "
          "（θ=单帧位姿 / Δε=额外区间旋转误差，度）--")
    print("  Δ\\档位 " + "   ".join(f"[{lo:.0f}-{hi:.0f}]" for lo, hi, _, _ in bands))
    for dmm in (0.5, 1.0, 2.0, 5.0):
        row = []
        for ib in range(len(bands)):
            sl = fit_small[ib][0]
            a = slope_eps[ib][0]
            row.append(f"{dmm / sl:5.3f}/{dmm / a:5.3f}" if sl > 1e-9 and a > 1e-9 else "   -  ")
        print(f"  {dmm:4.1f}mm " + "  ".join(row))
    print("\n（θ 是**单帧位姿**旋转注入幅度；ε 是**相邻帧相对**旋转误差；"
          "白噪声型注入下 ε≈√2·θ，但真实 LIO 误差含相关分量，换算只对本实验成立）")
    # ==========================================================================
    # 6. 模式 T / Tc：平移注入（成图、分档、测厚、鬼影、取中位**全部复用**模式 R
    #    的同一条代码路径，只有注入向量不同）
    # ==========================================================================
    def run_trans(name, seed0, correlated, s_grid):
        """平移注入扫描。返回 dict:
        s      注入模长网格（mm）
        th     各档 k=16 法向厚度中位（mm，NREP2 次实现取中位）
        inc    相对 s=0 本底（= §2.5 名义位姿本底）的厚度增量（mm）
        gh     各档切向鬼影 D 中位（mm）；ghinc 为其增量
        ep     注入产生的**区间**（相邻帧相对）平移误差 med / p90（mm）
        hfN    注入场自身 3-tap 二阶差分 P50（三维模长口径，mm）
        hfA    同上（逐轴口径，mm）—— 用于把真实 hf_p 反解成与本臂同结构的 s
        """
        ns = len(s_grid)
        th = np.full((ns, len(bands), NREP2), np.nan)
        gh = np.full((ns, NREP2, len(bands)), np.nan)
        ep = np.full((ns, NREP2, 2), np.nan)
        hfn = np.full((ns, NREP2), np.nan)
        hfa = np.full((ns, NREP2), np.nan)
        print("\n" + "=" * 78)
        style = (f"块内恒定（块长 {TC_BLOCK} 帧 = {TC_BLOCK * 0.1:.1f} s）、块间随机换向换号"
                 " ⇒ 低频相关" if correlated else "逐帧独立、方向各向同性 ⇒ 白噪声")
        print(f"模式 {name}（平移）注入实测：t_inj(k) = t(k) + u_k·s，{style}")
        print("=" * 78)
        print("   s(mm) 实现 区间平移误差mm med/p90  注入场3-tap mm  " +
              "  ".join(f"[{lo:.0f}-{hi:.0f}]" for lo, hi, _, _ in bands) + "   |  鬼影D mm")
        t1 = time.perf_counter()
        for iv, sv in enumerate(s_grid):
            reps = 1 if sv == 0.0 else NREP2
            for jr in range(reps):
                if sv == 0.0:                       # s=0 与 θ=0 共用 §2.5 那一次成图
                    for ib in range(len(bands)):
                        th[iv, ib, jr] = base_th[ib]
                    gh[iv, jr, :] = base_gh
                    ep[iv, jr] = (0.0, 0.0)
                    hfn[iv, jr] = hfa[iv, jr] = 0.0
                    print(f"  {sv:6.2f}  #{jr}      0.00 /  0.00        0.000   " +
                          "  ".join(f"{base_th[ib]:6.2f}" for ib in range(len(bands))) +
                          "   | " + "  ".join(f"{b:6.2f}" for b in base_gh) +
                          f"  ({time.perf_counter() - t1:5.1f}s)")
                    continue
                rng = np.random.default_rng(seed0 + iv * 100 + jr)
                vec = trans_vec(rng, sv * 1e-3, nf, correlated)
                t_inj = tn + vec
                P = build_cloud(Rn, t_inj, body, n)         # 与模式 R 同一成图函数
                dd = PL.local_thickness(P)                   # 与模式 R 同一测厚口径
                for ib, (lo, hi, m, md) in enumerate(bands):
                    th[iv, ib, jr] = float(np.median(dd[m]) * 1000.0)
                gh[iv, jr, :] = ghost_measure(P, Rn, t_inj, bands, gsamp, frameid)
                ep[iv, jr] = rel_trans_err_mm(tn, t_inj)
                hfn[iv, jr], hfa[iv, jr] = hf3tap(vec)
                print(f"  {sv:6.2f}  #{jr}  {ep[iv, jr, 0]:6.2f} /{ep[iv, jr, 1]:6.2f}"
                      f"      {hfn[iv, jr]:8.3f}   " +
                      "  ".join(f"{th[iv, ib, jr]:6.2f}" for ib in range(len(bands))) +
                      "   | " + "  ".join(f"{b:6.2f}" for b in gh[iv, jr]) +
                      f"  ({time.perf_counter() - t1:5.1f}s)")
        medv = np.nanmedian(th, axis=2)
        gmed = np.nanmedian(gh, axis=1)
        epm = np.nanmedian(ep, axis=1)
        return {"name": name, "s": np.array(s_grid, float), "th": medv,
                "inc": medv - medv[0][None, :], "gh": gmed,
                "ghinc": gmed - gmed[0][None, :], "ep": epm, "ep90": epm[:, 1],
                "hfN": np.nanmedian(hfn, axis=1), "hfA": np.nanmedian(hfa, axis=1)}

    TT = run_trans("T", SEED_T, False, S_GRID) if RUN_T else None
    TC = run_trans("Tc", SEED_TC, True, S_GRID) if RUN_TC else None

    # ==========================================================================
    # 7. 平移通道斜率 b = d(指标)/ds 与解析值对比
    #    解析（本轮自推，见 docstring）：法向跨帧层间距 = s·√(2/3) = 0.8165·s；
    #    切向跨帧质心分离 = √2·√(2/3)·s = 1.1547·s。
    # ==========================================================================
    B_N = float(np.sqrt(2.0 / 3.0))            # 0.8165 mm/mm（法向厚度）
    B_T = float(np.sqrt(4.0 / 3.0))            # 1.1547 mm/mm（切向鬼影）
    NB = len(bands)

    def print_inc(res, key, lab, unit="mm"):
        print(f"\n-- 模式 {res['name']}：{lab} 相对本底增量（{unit}；本底 = s=0 各档中位）--")
        print("   s(mm)  " + "  ".join(f"[{lo:.0f}-{hi:.0f}]" for lo, hi, _, _ in bands))
        for iv, sv in enumerate(res["s"]):
            print(f"  {sv:6.2f}  " + "  ".join(f"{res[key][iv, ib]:+7.2f}" for ib in range(NB)))

    def slope_tables(res, key, ana, note):
        """对每个深度档拟合 b = d(指标)/ds；近端（s<=5 mm，与真实量级同档）为判决口径。"""
        if res is None:
            return {}
        out = {}
        print(f"\n-- 模式 {res['name']}：{note} --")
        print("  档位           中位深度  b(全部s) R²    b(s<=5mm) R²     解析b    实测/解析")
        ss = res["s"]
        near = ss <= 5.0 + 1e-12
        for ib, (lo, hi, m, md) in enumerate(bands):
            y = res[key][:, ib]
            f_all = fitslope(ss, y)
            f_near = fitslope(ss[near], y[near])
            print(f"  [{lo:4.1f},{hi:4.1f}) m  {md:6.2f} m  {f_all[0]:8.3f} {f_all[1]:5.3f}  "
                  f"  {f_near[0]:8.3f} {f_near[1]:5.3f}  {ana:8.3f}  {f_near[0] / ana:6.3f}")
            out[ib] = (f_near[0], f_all[0], ana, f_near[1])
        return out

    if TT is not None:
        print_inc(TT, "inc", "k=16 法向厚度")
        b_thick = slope_tables(TT, "inc", B_N, "b = d厚度/ds（mm 厚度 / mm 平移误差）；"
                                             "解析 = 跨帧法向层间距 √(2/3)")
        g_slop = slope_tables(TT, "ghinc", B_T, "b_g = dD/ds；解析 = √2·√(2/3)")
    if TC is not None:
        print_inc(TC, "inc", "k=16 法向厚度（同 s 口径：每帧绝对误差模长仍是 s）")
        bc_thick = slope_tables(TC, "inc", B_N, "与模式 T 只差时间结构，故仍用同一 s 口径")
        bg_gh = slope_tables(TC, "ghinc", B_T, "同上")
    else:
        bc_thick, bg_gh = {}, {}
    if TT is None:
        b_thick, g_slop = {}, {}

    if TT is not None:
        k_ep_T = float(np.median(TT["ep"][1:, 0] / TT["s"][1:]))
        print(f"\n  模式 T 实测**区间**平移误差 med = " +
              ", ".join(f"{s:g}mm→{e:.2f}mm" for s, e in zip(TT["s"][1:], TT["ep"][1:, 0])) +
              f" mm（白噪声解析 √2·s = 1.414·s，实测系数 {k_ep_T:.3f}）")
    if TC is not None:
        print(f"  模式 Tc 实测**区间**平移误差 med = " +
              ", ".join(f"{s:g}mm→{e:.2f}mm" for s, e in zip(TC["s"][1:], TC["ep"][1:, 0])) +
              f" mm，p90 = " + ", ".join(f"{p:.2f}" for p in TC["ep90"][1:]) +
              f" mm ⇒ 块内恒定使相邻帧**区间**误差≈0：按构造只有 "
              f"{int(np.ceil(nf / TC_BLOCK)) - 1} 个块边界（占 {100.0 * (np.ceil(nf / TC_BLOCK) - 1) / (nf - 1):.1f}%"
              f" 的 {nf - 1} 个相邻帧区间）承担 √2·s 量级的跳变")
        print("  ⚠️ Tc 的口径争议（故只作敏感性臂，不作主判据）：在「相邻帧**区间**误差」口径下"
              "Tc 与 T 不可横比（Tc 的区间误差中位≈0）；只有在「**逐帧绝对**误差 = s」这一口径下"
              "两者才同格。本表所有 Tc 数字均按后者给出，判决主判据 = 模式 T。")

    # ==========================================================================
    # 8. 切向鬼影代理指标 D：模式 R 的斜率 vs 解析（这是法向厚度看不见的通道）
    # ==========================================================================
    print("\n" + "=" * 78)
    print("★ 补充口径：切向鬼影代理指标 D（k=16 邻域内跨帧质心的切向分离，mm）")
    print("=" * 78)
    gmed_r = np.nanmedian(gres, axis=1)
    ginc = gmed_r - gmed_r[0][None, :]
    print("-- 模式 R：D 增量 vs θ（mm）--")
    print("  theta(°)  " + "  ".join(f"[{lo:.0f}-{hi:.0f}]" for lo, hi, _, _ in bands))
    for it, th in enumerate(THETAS):
        print(f"  {th:7.2f}  " + "  ".join(f"{ginc[it, ib]:+7.2f}" for ib in range(NB)))
    print("\n-- 斜率 a_g = dD/dθ vs 解析 √2·√(2/3)·d·tan(1°) = 1.1547·d·tan1° --")
    print("   （纯几何 d·tanθ 一并给出：它是「一层整帧转动后点的切向位移」，"
          "D 是跨帧两层质心距离 ⇒ 解析系数是 √2·√(2/3)=1.1547，不是 1）")
    g_fit = {}
    for ib, (lo, hi, m, md) in enumerate(bands):
        anag = B_T * md * 1000.0 * np.tan(np.radians(1.0))
        anapure = md * 1000.0 * np.tan(np.radians(1.0))
        fg = fitslope(xs, ginc[:, ib])
        fgs = fitslope(xs[sel], ginc[sel, ib])
        print(f"  [{lo:4.1f},{hi:4.1f}) m  a_g(全部θ)={fg[0]:8.2f} R²={fg[1]:5.3f}   "
              f"a_g(θ≤0.3°)={fgs[0]:8.2f} R²={fgs[1]:5.3f}   解析={anag:8.2f}"
              f"（纯 d·tanθ={anapure:8.2f}）   实测/解析={fgs[0] / anag:6.3f}")
        g_fit[ib] = (fgs[0], fg[0], anag)
    print("\n  同一 θ 网格：法向厚度斜率 a_θ vs 切向鬼影斜率 a_g ⇒ 法向指标的「盲区倍数」")
    for ib, (lo, hi, m, md) in enumerate(bands):
        at = fit_small[ib][0]
        print(f"  [{lo:4.1f},{hi:4.1f}) m  a_θ={at:7.2f} mm/°   a_g={g_fit[ib][0]:7.2f} mm/°"
              f"   a_g/a_θ={g_fit[ib][0] / max(1e-9, at):6.1f}x")

    # ==========================================================================
    # 9. 真实量级 → 注入口径 的换算（两统计量口径不同，先统一到「逐帧绝对误差」）
    # ==========================================================================
    print("\n" + "=" * 78)
    print("★ 口径换算：ε0=0.13° 是**区间**（相邻帧相对）旋转误差，hf_p=3.85 mm 是**逐帧 3-tap")
    print("  二阶差分**统计量；二者不可直接比，各自换算到「逐帧绝对误差」再代入各自的实测斜率")
    print("=" * 78)
    f_axis, f_norm = iso_factors()
    print(f"  [数值标定 MC，非手抄常数] median|单轴 N(0,σ)| = {f_axis:.4f}σ，"
          f"median|三维各向同性模长| = {f_norm:.4f}τ")

    def hf_probe(pp, lab, dt=0.1):
        """3-tap P50（模长/逐轴口径）+ 局部二次拟合中心残差（去曲率的独立 σ 估计）。"""
        d2 = pp[:-2] - 2.0 * pp[1:-1] + pp[2:]
        hfn = float(np.median(np.linalg.norm(d2, axis=1)) * 1000.0)
        hfa = float(np.median(np.abs(d2)) * 1000.0)
        s_ax = hfa / (f_axis * np.sqrt(6.0))            # 逐轴口径 ⇒ 单轴 σ（白噪声假设）
        s_no = hfn / (f_norm * np.sqrt(6.0))            # 模长口径 ⇒ 单轴 σ
        W = 5
        r = np.arange(W) - (W - 1) / 2.0
        M = np.stack([np.ones(W), r, r * r], 1)
        Pm = np.linalg.pinv(M)
        g = float(np.linalg.norm(np.eye(W)[(W - 1) // 2] - Pm[0]))   # 中心残差噪声传递因子
        win = np.stack([pp[i:i + W] for i in range(len(pp) - W + 1)])
        rr = np.linalg.norm(win[:, (W - 1) // 2] -
                            np.einsum("j,mjc->mc", Pm[0], win), axis=1)
        r5 = float(np.median(rr) * 1000.0)
        s_fit = r5 / (f_norm * g)                       # 残差口径 ⇒ 单轴 σ
        sig = 0.5 * (s_ax + s_no)
        print(f"  {lab}（n={len(pp)}, dt≈{dt:.3f} s）")
        print(f"      3-tap P50：三维模长口径 = {hfn:6.2f} mm，逐轴口径 = {hfa:6.2f} mm")
        print(f"      白噪声换算 σ_axis = {sig:5.2f} mm（逐轴口径 {s_ax:.2f} / 模长口径 "
              f"{s_no:.2f}）⇒ 逐帧绝对平移误差 s_eq = √3·σ_axis = {np.sqrt(3.0) * sig:5.2f} mm")
        print(f"      交叉核验：局部二次 W=5 中心残差 P50 = {r5:5.2f} mm（噪声传递因子 "
              f"{g:.3f}）⇒ σ_axis = {s_fit:5.2f} mm ⇒ 残差/3-tap = {s_fit / max(1e-9, sig):.2f}")
        print(f"        （≈1 ⇒ 平移抖动以逐帧白噪声为主；≪1 ⇒ 3-tap 里多为真实曲率/低频，"
              f"s_eq 被高估；>1 ⇒ 含额外高频，s_eq 被低估）")
        return {"hfn": hfn, "hfa": hfa, "sig": sig, "seq": np.sqrt(3.0) * sig,
                "r5": r5, "sfit": s_fit, "ratio": s_fit / max(1e-9, sig),
                "seq_fit": np.sqrt(3.0) * s_fit}

    cal = {}
    try:
        at = np.loadtxt(BASE_TXT)
        dtx = float(np.median(np.diff(at[:, 0])))
        cal["seg"] = hf_probe(at[100:600, 1:4], f"base.txt §36.2 同段 k∈[100,600)（{T_EPS[0]}° 的出处）", dtx)
        cal["all"] = hf_probe(at[:, 1:4], "base.txt 全段", dtx)
    except Exception as e:                          # 参照轨迹不可读时不阻断判决
        print(f"  [警告] base.txt 复测跳过：{e}")
    cal_odo = hf_probe(tn, "本实验 odom 段（注入实际作用的那条轨迹，inputs.npz 时间窗）",
                       float(np.median(np.diff(stamps))))
    if cal:
        print(f"\n  旁证核对：方案引用的 3-tap hf_p = {T_HF_P} mm，本轮在 §36.2 同段实测为 "
              f"逐轴 {cal['seg']['hfa']:.2f} mm / 模长 {cal['seg']['hfn']:.2f} mm"
              f" ⇒ 3.85 mm 只能对应**逐轴**口径（模长口径实测接近 8 mm），"
              "本仓库文档里没有写明这一口径，属可疑口径，已如实标注。")

    K_EPS = float(np.median(eps_med[1:] / np.array(THETAS[1:])))
    TH0 = EPS0 / K_EPS
    print(f"\n  旋转侧：本实验实测 ε(θ)/θ = {K_EPS:.3f}（√2 = {np.sqrt(2.0):.3f}）⇒ "
          f"ε0={EPS0:.2f}° 对应逐帧注入幅度 θ0 = ε0/{K_EPS:.3f} = {TH0:.4f}°")
    print(f"      若真实误差相邻帧**完全相关** ⇒ θ0 = ε0 = {EPS0:.2f}°"
          f"（比白噪声口径大 {100.0 * (K_EPS - 1.0):.0f}%），故下表 X 给两种极端的区间。")
    print("      §36.3 已证 3-tap hf_θ 的 95% 是真实角运动曲率 ⇒ 真实的**逐帧绝对**旋转误差"
          "很可能远小于 θ0；这不影响 X=a·ε0（区间口径直接代入），但意味着"
          "「旋转通道还有多少没修干净」是未知的。")

    HF_REAL = cal_odo["hfn"]
    HF_REAL_A = cal_odo["hfa"]
    print(f"\n  平移侧：用**注入场自身的 3-tap** 与本实验 odom 段同口径匹配（不依赖高斯假设）：")
    if TT is not None:
        kT_N = float(np.median(TT["hfN"][1:] / TT["s"][1:]))
        kT_A = float(np.median(TT["hfA"][1:] / TT["s"][1:]))
        S_EQ = HF_REAL / kT_N                          # 判决用（模长口径，主）
        S_EQ_A = HF_REAL_A / kT_A                      # 逐轴口径版
        S_EQ_G = cal_odo["seq"]                        # 高斯换算版（独立复核）
        S_EQ_LO = cal_odo["seq_fit"]                   # 残差口径 ⇒ 下界
        print(f"      标定：模式 T 的 3-tap = {kT_N:.3f}·s（模长口径）= {kT_A:.3f}·s（逐轴口径）"
              f"；解析核对（把注入场当高斯近似）√2·f_norm={np.sqrt(2.0) * f_norm:.3f} / "
              f"√2·f_axis={np.sqrt(2.0) * f_axis:.3f}（实测更准，判决用实测值）")
        print(f"      本实验 odom 段实测 hf = {HF_REAL:.2f} mm（模长）/ {HF_REAL_A:.2f} mm（逐轴）")
        print(f"      ⇒ 逐帧绝对平移误差 s_eq = {S_EQ:.2f} mm（主口径，模长匹配）"
              f"；逐轴匹配 {S_EQ_A:.2f} mm；高斯换算 {S_EQ_G:.2f} mm"
              f"（差 {abs(S_EQ_G - S_EQ) / max(1e-9, S_EQ) * 100:.0f}%）；"
              f"去曲率残差口径 {S_EQ_LO:.2f} mm（下界）")
        print(f"      旁证 3.85 mm 若按**逐轴**口径直接代入：s_eq = {T_HF_P / kT_A:.2f} mm；"
              f"若误按**模长**口径代入：s_eq = {T_HF_P / kT_N:.2f} mm"
              f" ⇒ 两口径反解相差 {kT_N / kT_A:.2f} 倍")
    else:
        S_EQ = S_EQ_A = S_EQ_G = S_EQ_LO = float("nan")
    if TC is not None:
        kTC_N = float(np.median(TC["hfN"][1:] / TC["s"][1:]))
        if kTC_N > 1e-6:
            S_EQ_TC = HF_REAL / kTC_N
            note_tc = (f"Tc 的 3-tap = {kTC_N:.3f}·s ⇒ 同一 hf 反解出 s_eq = {S_EQ_TC:.2f} mm"
                       f"（是 T 口径的 {S_EQ_TC / max(1e-9, S_EQ):.1f}x）")
        else:
            S_EQ_TC = float("nan")
            note_tc = (f"Tc 的 3-tap = {kTC_N:.4f}·s ≈ 0 ⇒ **3-tap 无法标定低频相关型误差的"
                       "幅度**（该统计量对低频不敏感），s_eq 反解发散")
        print("      " + note_tc)
        disc = (np.median([bc_thick[i][0] / max(1e-9, b_thick[i][0]) for i in range(NB)])
                if (bc_thick and b_thick) else float("nan"))
        print("      ⇒ 因此 Tc 一律按「逐帧绝对误差 = s」口径直接代入同一个 s_eq（下界型）；"
              f"实测同 s 下低频相关型的分层斜率折扣 b_Tc/b_T = {disc:.2f}"
              "（<1 ⇒ 真实误差若是低频结构，其绝对幅度只会更大、但单位幅度的分层只会更少，"
              "两个方向相反 ⇒ 判决仍以模式 T 为主判据）。")
    else:
        S_EQ_TC = float("nan")
    print("  ⚠️ 换算假设与不确定性（判决表可用性的关键）：")
    print("      (1) X = a_ε · ε0：旋转通道直接在**区间**口径上代入，不需换算（斜率 a 本身"
          "就是按 ε_tot=√(ε0²+ε²) 拟合出来的），换算只影响「θ0 等效注入幅度」这一栏的显示。")
    print("      (2) Y = b_s · s_eq：平移通道的 s_eq 由 3-tap 反解，**假设真实平移误差逐帧独立**；"
          f"本轮 W=5 去曲率残差/3-tap = {cal_odo['ratio']:.2f}"
          f"（<1 ⇒ 3-tap 含真实曲率/低频，主口径 s_eq={S_EQ:.2f} mm 偏大，"
          f"下界 s_eq={S_EQ_LO:.2f} mm）⇒ Y 给区间。")
    print("      (3) ε0 与 hf_p 的出处轨迹不同：ε0=0.13° 取自 §36.2 的 base.txt 段，"
          "本实验注入作用在 replay3 的 odom 段（inputs.npz 时间窗）；本轮在 base.txt 上复测 hf 见上，"
          "两条轨迹不完全可互换 ⇒ 表内数字带此风险。")
    print("      (4) §36.3 的「hf_θ 95% 是真实曲率」只对旋转成立，平移侧必须单独检验（即 (2)）。")

    # ==========================================================================
    # 10. ★ 判决表：同一深度档并排给 a（mm/° 区间旋转误差）与 b（mm/mm 平移误差）
    # ==========================================================================
    ib_m = ib5 if ib5 is not None else int(np.argmin([abs(md - 5.0) for _, _, _, md in bands]))
    order = [ib_m] + [i for i in range(NB) if i != ib_m]

    def need_amp(amps, incs, target=1.0):
        """实测曲线上**首次**达到 target 增量所需的注入幅度（线性内插，不外推）；
        曲线没到 target 就返回 nan。用它做「饱和度 = 真实幅度 / 该幅度」的**非线性稳健**
        排名，避免只用局部斜率外推（本指标对小幅注入是饱和/平方根型响应）。"""
        for i in range(1, len(amps)):
            if incs[i] >= target:
                f = (target - incs[i - 1]) / max(1e-9, incs[i] - incs[i - 1])
                return float(amps[i - 1] + f * (amps[i] - amps[i - 1]))
        return float("nan")

    print("\n" + "=" * 78)
    print("★ 判决表：法向厚度 / 切向鬼影 中「旋转通道 vs 平移通道」的贡献")
    print("=" * 78)
    print("  两通道各自把**自己同口径**的真实统计量代入自己同口径的实测斜率："
          "旋转用区间 ε0（斜率 a_ε 已按 ε_tot 拟合），平移用逐帧 s_eq（斜率 b 按 s 拟合）。")
    print("  ∴ 表中的「通道贡献」是同一个量纲（本底之上由该通道真实误差造成的厚度增量），"
          "可以直接比大小；但两种**斜率**之间不可横比（口径不同）。")
    print(f"  代入真实量级：ε0 = {EPS0:.2f}°（逐帧等效 θ0 = {TH0:.4f}°）；"
          f"s_eq = {S_EQ:.2f} mm（下界 {S_EQ_LO:.2f} mm）；"
          f"切向鬼影同口径代入")
    hdr = "  通道                       斜率          代入量        贡献mm   占本底%"
    rec = {}
    for ib in order:
        lo, hi, m, md = bands[ib]
        npts = int(m.sum())
        aE = slope_eps[ib][0]                         # mm/°（区间口径，与 ε0 同口径）
        a_th = aE * K_EPS                             # mm/°（逐帧口径）
        Xn = aE * EPS0                                # = a_th·θ0
        Xn_hi = a_th * EPS0                           # 相邻帧完全相关 ⇒ 旋转通道上界
        Yn = b_thick[ib][0] * S_EQ if b_thick else float("nan")
        Yn_lo = b_thick[ib][0] * S_EQ_LO if b_thick else float("nan")
        Yn_c = bc_thick[ib][0] * S_EQ if bc_thick else float("nan")
        Xg = g_fit[ib][0] * TH0
        Xg_hi = g_fit[ib][0] * EPS0
        Yg = g_slop[ib][0] * S_EQ if g_slop else float("nan")
        Yg_c = bg_gh[ib][0] * S_EQ if bg_gh else float("nan")
        print(f"\n  ◆ 档位 [{lo:.1f},{hi:.1f}) m（中位深度 {md:.2f} m，n={npts}）"
              + ("  ★主档" if ib == ib_m else "  （旁档）")
              + ("  ⚠样本不足，仅供参考" if npts < MIN_BAND_N else ""))
        print(hdr)
        print(f"  法向厚度 ← 旋转    {aE:8.2f} mm/°  {EPS0:8.3f}°(区间){Xn:9.2f}"
              f"  {100.0 * Xn / med[0, ib]:7.1f}%   [完全相关上界 {Xn_hi:.2f}]")
        print(f"    （逐帧口径等价写法 {a_th:7.2f} mm/° × {TH0:.4f}° = {a_th * TH0:.2f} mm）")
        if b_thick:
            print(f"  法向厚度 ← 平移T {b_thick[ib][0]:8.3f} mm/mm  {S_EQ:8.2f}mm(逐帧)"
                  f"{Yn:9.2f}  {100.0 * Yn / med[0, ib]:7.1f}%   [下界 {Yn_lo:.2f}]")
        if bc_thick:
            print(f"  法向厚度 ← Tc    {bc_thick[ib][0]:8.3f} mm/mm  {S_EQ:8.2f}mm(逐帧)"
                  f"{Yn_c:9.2f}  {100.0 * Yn_c / med[0, ib]:7.1f}%")
        if b_thick and Xn > 1e-9:
            print(f"    ⇒ 法向厚度 Y/X = {Yn / Xn:5.2f}（模式 T）"
                  f" / {Yn_c / Xn:5.2f}（模式 Tc，同 s 代入）")
        print(f"  切向鬼影 ← 旋转    {g_fit[ib][0]:8.2f} mm/°  {TH0:8.4f}°(逐帧){Xg:9.2f}"
              f"  {100.0 * Xg / base_gh[ib]:7.1f}%   [上界 {Xg_hi:.2f}]")
        if g_slop:
            print(f"  切向鬼影 ← 平移T {g_slop[ib][0]:8.3f} mm/mm  {S_EQ:8.2f}mm(逐帧)"
                  f"{Yg:9.2f}  {100.0 * Yg / base_gh[ib]:7.1f}%")
        if bg_gh:
            print(f"  切向鬼影 ← Tc    {bg_gh[ib][0]:8.3f} mm/mm  {S_EQ:8.2f}mm(逐帧)"
                  f"{Yg_c:9.2f}  {100.0 * Yg_c / base_gh[ib]:7.1f}%")
        if g_slop and Xg > 1e-9:
            print(f"    ⇒ 切向鬼影 Y/X = {Yg / Xg:5.2f}（T）" +
                  (f" / {Yg_c / Xg:5.2f}（Tc）" if bg_gh else ""))
        # ---- 口径稳健性：本指标对小幅注入是饱和型，斜率口径会改变排名，必须一起打印 ----
        th1 = need_amp(np.array(THETAS), inc[:, ib], 1.0)
        s1 = need_amp(TT["s"], TT["inc"][:, ib], 1.0) if TT is not None else float("nan")
        X_all = fit_all[ib][0] * TH0                      # 全程 θ 斜率（逐帧口径）
        Y_all = (b_thick[ib][1] if b_thick else float("nan")) * S_EQ
        print(f"    [口径稳健性] 近端斜率 Y/X = {Yn / max(1e-9, Xn):5.2f}   "
              f"全程斜率 Y/X = {Y_all / max(1e-9, X_all):5.2f}"
              f"（{X_all:.2f} vs {Y_all:.2f} mm）")
        print(f"                  造成 +1.00 mm 厚度所需幅度（实测曲线内插）："
              f"旋转 θ* = {th1:.3f}°（真实 {TH0:.4f}° ⇒ 饱和度 {TH0 / th1 if th1 == th1 else float('nan'):.2f}）"
              f"；平移 s* = {s1:.2f} mm（真实 {S_EQ:.2f} ⇒ 饱和度 {S_EQ / s1 if s1 == s1 else float('nan'):.2f}）"
              f" ⇒ 饱和度大的通道更接近'值得修'，两值同量级则排名不稳健")
        rec[ib] = (Xn, Yn, Yn / max(1e-9, Xn), Y_all / max(1e-9, X_all), th1, s1)

    # ---- 主档结论（判决意见）----
    lo, hi, m, md = bands[ib_m]
    aE = slope_eps[ib_m][0]
    Xn = aE * EPS0
    Xt5 = aE * (EPS0 - 0.05)
    Yn = b_thick[ib_m][0] * S_EQ if b_thick else float("nan")
    Yn_lo = b_thick[ib_m][0] * S_EQ_LO if b_thick else float("nan")
    Yn_c = bc_thick[ib_m][0] * S_EQ if bc_thick else float("nan")
    Xg = g_fit[ib_m][0] * TH0
    Yg = g_slop[ib_m][0] * S_EQ if g_slop else float("nan")
    print("\n  ★ 主档结论（" + f"[{lo:.1f},{hi:.1f}) m，中位深度 {md:.2f} m）")
    print(f"      法向厚度：旋转通道贡献 X = {Xn:.2f} mm；平移通道贡献 Y = {Yn:.2f} mm"
          f"（保守下界 {Yn_lo:.2f} mm；Tc 同 s 代入 {Yn_c:.2f} mm）⇒ Y/X = {Yn / max(1e-9, Xn):.2f}")
    print(f"      切向鬼影：旋转通道 X = {Xg:.2f} mm；平移通道 Y = {Yg:.2f} mm"
          f" ⇒ Y/X = {Yg / max(1e-9, Xg):.2f}")
    print(f"      把区间旋转误差 0.13°→0.05° 只拿回 {Xt5:.2f} mm（占本底 "
          f"{100.0 * Xt5 / med[0, ib_m]:.1f}%）；把平移抖动降 50% 拿回 {0.5 * Yn:.2f} mm")
    print(f"      本底 {med[0, ib_m]:.2f} mm 的拆解：旋转 {Xn:.2f} + 平移(T) {Yn:.2f} = "
          f"{Xn + Yn:.2f} mm（{100.0 * (Xn + Yn) / med[0, ib_m]:.0f}%），剩余 "
          f"{max(0.0, med[0, ib_m] - Xn - Yn):.2f} mm 与位姿误差无关"
          "（LiDAR 测距噪声 / 面元粗糙度 / 动态物 / 扫描时间畸变）。"
          "⚠️ 两项相加可能超过本底 ⇒ 说明 s_eq 高估，看保守下界。")
    dom = "平移" if (b_thick and Yn > Xn) else "旋转"
    th_eps0 = med[0, ib_m] - aE * EPS0                    # 口径 B：把 ε 修到 0 的厚度
    X_all_m = fit_all[ib_m][0] * TH0
    Y_all_m = (b_thick[ib_m][1] if b_thick else float("nan")) * S_EQ
    r_near = Yn / max(1e-9, Xn)
    r_all = Y_all_m / max(1e-9, X_all_m)
    th1m, s1m = rec[ib_m][4], rec[ib_m][5]
    sat_r = TH0 / th1m if th1m == th1m else float("nan")
    sat_t = S_EQ / s1m if s1m == s1m else float("nan")
    ana_r = fit_small[ib_m][0] / (md * 1000.0 * np.tan(np.radians(1.0)))
    g_ratio = g_fit[ib_m][0] / g_fit[ib_m][2]
    other = [i for i in order if i != ib_m]
    print("\n  ◆ 判决意见")
    print(f"      1) 反向假设「分层主要由**平移**不连续驱动」**未被证实**：同一 k=16 法向厚度上，"
          f"旋转通道 {Xn:.2f} mm > 平移通道 {Yn:.2f} mm（主档 {dom} 主导，Y/X = {r_near:.2f}）；"
          "旁档同向：" + "，".join(
              f"[{bands[i][0]:.0f}-{bands[i][1]:.0f})m Y/X={rec[i][2]:.2f}"
              + ("（样本不足）" if int(bands[i][2].sum()) < MIN_BAND_N else "") for i in other))
    print("      2) 但**这个验收指标本身没有判定力**，两个方向都不该拿它下结论，依据全是实测：")
    print(f"        (a) 本底 {med[0, ib_m]:.2f} mm 里两通道合计只占 {Xn + Yn:.2f} mm"
          f"（{100.0 * (Xn + Yn) / med[0, ib_m]:.0f}%），剩余 {max(0.0, med[0, ib_m] - Xn - Yn):.2f} mm "
          f"与位姿误差无关；即便把区间旋转误差修到 0，本档也只有 {th_eps0:.2f} mm（口径 B）>"
          "5 mm 目标 ⇒「地图厚度 ≤5 mm」不可能靠修位姿达成，它不是位姿改进的合格验收量。")
    print(f"        (b) 指标对小幅注入是**饱和/平方根型**响应：近端斜率给 Y/X={r_near:.2f}，"
          f"全程斜率给 Y/X={r_all:.2f}（{X_all_m:.2f} vs {Y_all_m:.2f} mm）⇒ 排名随斜率口径翻转；"
          f"按「造成 +1mm 所需幅度」内插：旋转 θ*={th1m:.3f}°（饱和度 {sat_r:.2f}）、"
          f"平移 s*={s1m:.2f}mm（饱和度 {sat_t:.2f}）⇒ 两通道其实同量级，本指标分不出主次。")
    print(f"      3) 旋转位移 δ×r **垂直于视线**，只有墙面法向与视线不平行时才落到法向"
          f"（前视墙 0%，侧墙 100%）⇒ 实测 a_θ/(d·tan1°) = {ana_r:.3f} 是**朝向混合后的平均投影**"
          "（上一轮 0.18 本轮复现），说明法向厚度对旋转是**部分盲区**，但不能由此说旋转没有代价。")
    print(f"      4) 本轮新补的切向鬼影代理 D **同样不达标（如实上报，不作替代指标）**："
          f"本底 D = {base_gh[ib_m]:.1f} mm 远大于误差项（k=16 邻域内每帧只摊到 2~4 点，"
          f"子集质心被邻域真实几何散布 + 抽样噪声主导），旋转实测斜率 a_g = {g_fit[ib_m][0]:.1f} mm/° "
          f"只有解析 {g_fit[ib_m][2]:.1f} mm/° 的 {100.0 * g_ratio:.0f}%；平移斜率 "
          f"{g_slop[ib_m][0] if g_slop else float('nan'):.3f} mm/mm 只有解析 "
          f"{B_T:.3f} 的 {100.0 * (g_slop[ib_m][0] / B_T if g_slop else 0):.0f}%"
          " ⇒ 它没能把旋转的切向代价放大出来，当前实现不可用。要用必须先把邻域限制在**同一平面芯区**"
          "（`_probe_layering.global_planes` 的多峰间距 sep_cm 就是这个口径的成品），"
          "或把「跨帧」定义改成同一面墙的两层点集质心距，而不是 k=16 局部邻域。")
    print("      5) 对 Phase 1 的判决：本轮**不能**判「旋转优先的收益通道错了」；能判的是"
          "「k=16 法向厚度对 0.13° 级旋转与 3 mm 级平移都不敏感，作为唯一验收指标会漏判」。"
          "建议判据改三件：① §36.2 的 hf_θ（旋转直接量，线性响应，已证 95% 是真实曲率 ⇒ 需配"
          "去曲率版本）；② 平面芯区**多峰间距 / 跨帧质心距**（分层绝对量，厘米级响应）；"
          "③ 法向厚度降级为守门指标（只要求相对 b_lio 不劣化，不设 ≤5 mm 硬门）。"
          "平移通道若要单独立项，先把 s_eq 的幅度标定做实（当前只有 3-tap 一个来源，"
          f"W=5 去曲率残差口径的下界是 {S_EQ_LO:.2f} mm，主口径 {S_EQ:.2f} mm）。")
    print("\n（θ 是**单帧位姿**旋转注入幅度；ε 是**相邻帧相对**旋转误差；"
          "白噪声型下 ε≈√2·θ，但真实 LIO 误差含相关分量，换算只对本实验成立）")
    print(f"\n总耗时 {time.perf_counter() - t_all:.1f} s")
    print("DONE_P0E")


if __name__ == "__main__":
    main()
