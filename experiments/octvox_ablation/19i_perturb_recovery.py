#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P0-F：把"已知大小的旋转扰动"注入现有 LIO 轨迹，看滑窗+纯旋转因子图能修掉多少。

【目的】
  P0-C/P0-D（exp_records §36.2/§36.5）证明"图像相邻帧相对旋转"通道具备独立观测能力
  （vs IMU 随机误差 0.048 deg、随机 0.043 deg，自报 sigma 约 0.005 deg），但没有证明
  "修完之后轨迹真的变好、且不伤真实运动"。P0-F 用真值已知的注入扰动回答三问：
    (1) 抑制比 dB = 10*log10( sum||delta||^2 / sum||delta - dtheta||^2 )：注入 delta 后
        修正量 dtheta 能压掉多少扰动能量；
    (2) 残余：修正后相对真值参照的残余扰动 med/p90，以及修正后区间残差 vs IMU 参照
        还剩多少 deg（"是否更接近现实"的主判据）；
    (3) 真实运动保持度：注入一段真实量级的低频运动（0.2 Hz / 0.5 deg 正弦）之后，
        修正是保住它还是当噪声抹掉。保留率明显 <100% => 判"把真实运动抹掉了"。
  另跑**纯平滑对照**（EMA 因果 / 中心窗固定延迟）：若修正器不显著优于纯平滑，整个
  图像方案应降级为"地图层接个平滑位姿"（判决门明确写了这条）。

【口径（与 §36 对齐才可比）】
  * V.R_CL 实测是 b->c（v_c = R_CL v_b）=> R_wc = R_wb R_CL^T；body 系区间相对旋转映到
    相机系统一走 b2c_b(E) = R_CL E R_CL^T。
  * 曝光中点相对图像戳滞后 DT0 = 0.110 s（§36.1 实测，IMU/LIO 两参照同点复现）；取 LIO
    位姿一律在连续轨迹上 slerp 插值（V.Traj.at），不取最近帧。
  * 视觉旋转估计 = 深度辅助 6-DOF LM `RT.estimate_rotation_d`（tc 取自 LIO、逆深度平面
    参数化自由）；不用裸 Procrustes / 5-point（§36.2：低视差下退化，0.35 deg 是退化不是
    噪声）。协方差直接用求解器自报 (J^T J)^-1（§36.5 实测比值 1.34），再乘实测 chi2 膨胀
    系数 kappa（本脚本自己标定并打印）。
  * 修正只看旋转：R'_k = Exp(dtheta_k) R_pert(k)；平移沿用 LIO 不动（§36.4 证明该视觉
    因子对平移模长零观测能力）。

【本实验在方法上成立的前提（改代码前务必读懂）】
  R_vis(k->k+1) 只由图像对应点 + 一个 tc 向量决定，与注入的旋转扰动无关：
    - 图像对应点与 LIO 位姿无关；
    - tc 是 LIO 的平移量；本实验只扰旋转、不扰平移，故 tc 不变。若在线系统里 tc 被扰动
      过的姿态转过 delta，按 §36.4 实测敏感度（方向转 10 deg => 注入 1sigma=0.043 deg）
      折算，delta <= 0.2 deg 的泄漏 <= 0.0009 deg，比 0.05 deg 目标小两个量级，可忽略。
  => 注入扰动后**不需要重测视觉观测**：测量只做一次并缓存 /tmp npz，扫参只重解修正器。

【"干净参照"的严格含义（别读错判决）】
  R_clean 就是被扰动的这条 LIO 轨迹本身。它相对 IMU 有 0.13 deg 量级、相对图像有 0.13 deg
  量级的区间误差，因此：
    - 判"恢复力"用真值已知的 delta（dB / 残余 med），这是主判据；
    - "修正后区间残差 vs R_clean" 有不可约地板 = 本段实测"图像 vs LIO"残差，拿它跟
      0.05 deg 目标直接比属于口径错误，脚本把地板单独打印；
    - 判"修完是否更接近现实"用 vs IMU。
  视觉因子只约束相对旋转 => 窗口内"所有位姿同转一个常值"完全不可观：delta 的共模/低频
  部分修不掉是**结构性事实**，不是实现缺陷；脚本用 cm 列如实报告残余共模。

【第二轮复核（本版本）：为什么要重跑，以及新增的六张表】
  第一版把负结果的根因写成"信息量不足（视觉噪声 0.1409 deg > 要修的 0.05 deg）"，
  这个归因**错了**，被同一份输出里的三个数当场证伪：
      图像 vs IMU 0.0283 << LIO vs IMU 0.1299，且 0.1409 ≈ sqrt(0.0283^2+0.1299^2)
  => 0.1409 里绝大部分是**基线自己的误差**。而 `Log(V_k^T * R'_interval)` 这个残差里，
     对未知量 dtheta 而言噪声项只有视觉自身误差；(LIO 误差) 那一项是**待求的信号**。
     第一版用"图像 vs LIO"标定 kappa=42，等于把待消除的信号当成噪声，于是先验支配、
     dtheta->0、dB≈0，看起来"修不动"。本版按下面五步实测排除/坐实：
   表 0（交付①）纯图像链基线：把相邻帧视觉相对旋转**直接链式积分**成一条轨迹
        （= 先验权重->0、视觉因子当硬约束的极限解），测它的区间误差 vs IMU 与跨 N 帧
        累积误差。它给出修正器"原则上可达"的天花板：区间级 ≈ 0.03 deg ⇒ 上一轮的负
        结果是加权伪影；区间级 >= 0.13 deg ⇒ 链式累积真的是信息上限，负结果成立。
   表 A/A2（交付③）三种因子噪声口径并排跑同一张 lambda0 x lambda2 网格，并补
        clip>=0.5/1.0 与 lambda2=off，每档打印 S1/S2 激活次数：
          W1 kappa_vl  = 自报协方差 x 以"图像 vs LIO"标定的 kappa（第一版标称，伪影源）
          W2 kappa_vi  = 自报协方差 x 以"图像 vs IMU"标定的 kappa（剥掉基线误差，保留
                         逐因子协方差形状信息）—— 交付③要求的"视觉本征误差"口径
          W3 sigma_vi  = 直接用作本段实测的"图像 vs IMU"误差协方差（各向同性、不看自报
                         协方差），对自报协方差形状是否可信做鲁棒性检验
   表 C（交付④）无稳态延迟纯平滑基线：对区间旋转向量做**局部二次(Savitzky-Golay)**
        拟合（win 5/7/9，二次运动零稳态误差），出"离线零相位"与"固定延迟"两口径；
        第一版的 EMA/中心窗基线自带群延迟，L0 零扰动就有 0.6 deg 级"失真"，判决门
        "显著优于纯平滑"过得廉价，不算证据。
   表 D（交付②）去共模的 vs IMU **区间**误差：先数值证明"区间级 vs IMU 对 dtheta 的
        共模严格不变"（把共模精确左乘进去，imu 列变化应为 0），再按"逐轴减均值 + 减
        一次趋势项"剥掉图像/IMU 两通道共享的系统性分量，重报 基线 -> 修正后。
   表 E（判 (a)/(b)/(c) 的决定性一步）良定义复测：第一版的 res/dB 是**相对 R_clean
        （=LIO 本身）**算的，而 LIO 自带 0.13 deg 区间误差 > 注入的 0.10 deg，于是
        "把基线真实误差也修掉了"会被 res 记成"误修"，该门在 eps(基线误差) > delta(注入)
        时**根本不可辨**。表 E 先用无扰动(L0)修正器造一条"干净基线"R_base（其区间误差
        vs IMU 单独实测、作为新的不可约地板），再把同样的 delta 注入到 R_base 上重跑，
        此时的 res/dB 才是"扰动恢复力"的良定义度量。
   表 E2（比表 E 更干净一问）把扰动注入到**纯图像链**（表 0，未做任何优化）上再修：
        图像链的区间按构造就等于 R_vis，未扰动时视觉残差 ~0，故残差里的唯一信号就是
        注入的 delta 本身，res = ||delta - dtheta|| 完全不掺"该往图像拉"的真实分量。
        这一档才真正回答父代理的质疑"求解器根本没在修"；表 E 的 R_base 仍被图像拉着，
        其 res 同口径下依然不可辨，故只作记录、不参与归因判据。

【第二轮实测前修掉的三处口径/数值缺陷（都会改变地板数字，务必按本轮实测值取数）】
  A) ang_pair 原先直接 arccos(clip(0.5*(trace-1)))。V.R_CL 的正交精度只有 ~1e-6，
     经 b2c_b 后区间矩阵乘积的 trace 会超出 3（实测 +8.6e-7），clip 把它截成 1 =>
     **<=0.1 deg 的角度被系统性误报为 0**（实测同一批矩阵：trace 版 0.0000 deg、
     log 向量模长版 0.0541 deg）。现改为先 so3_proj（批量 SVD 极分解）再取 arccos。
     => 第一版的三个地板 0.0283 / 0.1299 / 0.1409 全部被低估，本轮值才是可引用的。
  B) W3（视觉本征协方差）的样本 g_vi 多乘了一次 D2R（log_b 已是弧度），协方差被缩小
     3283 倍、三个特征值全部落到 SIG_FLOOR，W3 实际等价于"sigma=0.020 deg 的常数加权"
     而不是"本段实测的视觉本征误差"。去掉该乘法后 W3 才真正是实测口径。
  C) 新增极限口径 W4（kappa=1：完全信任求解器自报协方差、不做 chi2 膨胀），即"视觉因子
     当近乎硬约束"，与表 0 的纯图像链是同一极限，用来量"先验完全不挡路时修正器能走多远"。
     第一版没有这一档，故无法区分"先验挡路"与"信息不够"。
  D) 表头打印注入 RMS 时原先取 make_delta 的 info['rms']，而该 info 只有 L4 档才有 amp/
     freq，其余档返回空字典 -> KeyError；改为 d_rms_of(level) 由真值 delta 现算。smoke 模式
     fine_c 退化为单元素，表 A/A2 里的 fine_c[1] 越界 -> 统一改用 CLIP_MAIN。

【实现选择：滑窗 vs 整段批求解（二选一，这里两个都做但主次分明）】
  主表用**在线因果滑窗**（W=15，逐帧解一次窗口、只取最新帧 dtheta_k，零额外延迟）——
  因为 Phase 1 落地的是小滑窗，只有它能报在线口径；整段稀疏批处理作为"离线最优上界"
  另出一列，用来量滑窗离全局最优还差多少（尤其低频真实运动的保真度）。两者不得横比延迟，
  表头分别标注。
  求解器是自写的向量化 Levenberg-Marquardt（解析 Jacobian + 启动时中心差分自检 +
  Huber/IRLS 外循环），不用 scipy.optimize.least_squares：本网格约 12 万次窗口求解，
  scipy 的数值 Jacobian + 调用开销放不下。若改用 least_squares，注意 method='lm'
  不能配 loss=。

【用法】
  python3 19i_perturb_recovery.py                      # 默认 Wn=350 K0=500 全流程
  python3 19i_perturb_recovery.py > /tmp/p0f2.log 2>&1 # 第二轮复核完整 stdout
  python3 19i_perturb_recovery.py 60 100 --smoke       # 小样本调通（缩网格）
  python3 19i_perturb_recovery.py 350 500 --keep       # 保留 /tmp 测量缓存便于复跑
  python3 19i_perturb_recovery.py 60 100 --smoke --serial   # 关掉并行，验证数字一致
  完整 stdout：python3 19i_perturb_recovery.py > /tmp/p0f.log 2>&1
  预算：配置级 fork 进程池（<=12 worker）并行，网格不裁；建议带 OMP_NUM_THREADS=1
  以免 BLAS 线程互相抢核。
"""
import multiprocessing as mp
import os
import sys
import time

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

sys.path.insert(0, '/home/ywj/git/Super-LIVO/experiments/octvox_ablation')
import _p0_bag as B            # noqa: E402
import _p0_visual as V         # noqa: E402
import _p0_rot as RT           # noqa: E402

BASE = '/home/ywj/slam_data/retail_abl/base.txt'
DT0 = 0.110          # 曝光中点相对图像戳的滞后（§36.1 实测）
MINBASE = 0.008      # 深度辅助模型的基线（|tc|）下限，与 19e/19f 同口径
W_SLIDE = 15         # 在线滑窗帧数（= Phase 1 C++ 的 frames）
SEED = 42            # RNG 主种子（可复现）
D2R = np.pi / 180.0
RAD = 180.0 / np.pi

LEVELS = ('L0-zero', 'L1-iso0.05', 'L2-rand0.10', 'L3-alt0.20', 'L4-sine+noise')
LEVEL_SEED = {'L0-zero': 0, 'L1-iso0.05': 101, 'L2-rand0.10': 202,
              'L3-alt0.20': 303, 'L4-sine+noise': 404}    # 固定子种子（不用 hash）
LEVEL_DESC = {
    'L0-zero': '不注入（量修正器自身的"误修量"）',
    'L1-iso0.05': '0.05 deg：固定模长 + 每帧随机方向（各向同性白噪声）',
    'L2-rand0.10': '0.10 deg：逐帧三轴高斯（总 RMS=0.10）',
    'L3-alt0.20': '0.20 deg：固定随机轴、符号逐帧 ±1 交替（检验钳位/帧间连续）',
    'L4-sine+noise': '真实运动：0.2 Hz / 0.5 deg 正弦绕世界 Z + 0.1 deg 白噪声',
}
HUBER_C = 2.5        # 视觉 Huber 阈值（作用于归一化残差范数；3 自由度 chi2 ~ 2.4）
SLEW_FACTOR = 2.0    # S2：slew = 2 * clip_rot

TARGET = 0.05        # 判决：残余扰动 med <= 0.05 deg
RET_MIN = 80.0       # 判决：0.2 Hz 真实正弦幅值保留率 >= 80 %
GRID_SIG_P = (0.03, 0.10, 0.30, 1.00)   # lambda0 一阶先验 sigma (deg)
GRID_SIG_D2 = (0.10, 0.50, None)        # lambda2 二阶差分 sigma (deg)，None=不加
GRID_CLIP = (0.10, 0.30)                # S1 绝对钳位 clip_rot (deg)
SIG_FLOOR = 0.02     # 我加的：特征值下限对应的 sigma (deg)，防退化轴权重爆炸
LINES = 118
SPANS = (1, 2, 5, 10, 20, 50)      # 表 0 的累积误差跨帧数（§36.2 口径）
# 第二轮的网格：主表三口径并排（窄网格），细扫只在决定性口径上做全网格。
MAIN_SP = (0.10, 0.30, 1.00)       # 表 A 的 lambda0
MAIN_SD = (0.50, None)             # 表 A 的 lambda2
FINE_CLIP = (0.10, 0.30, 0.50, 1.00)   # 表 A2 的 clip 全档
SG_WINS = (5, 7, 9)                # 表 C 的 Savitzky-Golay 窗长（奇数，局部二次）
REF_362 = {2: 0.081, 20: 0.416}   # §36.2 已记录的链式累积误差，仅**对照**（非本次实测）


# --------------------------------------------------------------------------- #
# 批量 SO(3) 数学（内部一律弧度）
# --------------------------------------------------------------------------- #
def skew_b(W):
    """(m,3) -> (m,3,3) 反对称阵。"""
    K = np.zeros((len(W), 3, 3))
    K[:, 0, 1] = -W[:, 2]; K[:, 0, 2] = W[:, 1]
    K[:, 1, 0] = W[:, 2];  K[:, 1, 2] = -W[:, 0]
    K[:, 2, 0] = -W[:, 1]; K[:, 2, 1] = W[:, 0]
    return K


def exp_b(W):
    """(m,3) -> (m,3,3) Rodrigues。"""
    th = np.linalg.norm(W, axis=1)
    small = th < 1e-8
    t = np.where(small, 1.0, th)
    K = skew_b(W)
    return (np.eye(3)[None] + (np.sin(t) / t)[:, None, None] * K
            + ((1.0 - np.cos(t)) / t ** 2)[:, None, None] * (K @ K))


def log_b(R):
    """(m,3,3) -> (m,3) 弧度 log。"""
    th = np.arccos(np.clip(0.5 * (np.trace(R, axis1=1, axis2=2) - 1.0), -1.0, 1.0))
    w = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0],
                  R[:, 1, 0] - R[:, 0, 1]], axis=1)
    small = th < 1e-8
    s = np.where(small, 0.5, th / (2.0 * np.sin(np.where(small, 1.0, th))))
    return w * s[:, None]


def jl_b(P):
    """(m,3) -> (m,3,3) 左 Jacobian：d log(Exp(a)Exp(p))/da|_{a=0}。"""
    th = np.linalg.norm(P, axis=1)
    small = th < 1e-8
    t2 = np.where(small, 1.0, th ** 2)
    t3 = np.where(small, 1.0, th ** 3)
    K = skew_b(P)
    a = np.where(small, 0.5 - th ** 2 / 24.0, (1.0 - np.cos(th)) / t2)
    b = np.where(small, (1.0 - th ** 2 / 20.0) / 6.0, (th - np.sin(th)) / t3)
    return np.eye(3)[None] - a[:, None, None] * K + b[:, None, None] * (K @ K)


def inv_sqrt_sym(S, floor):
    """PSD 3x3 的逆平方根（特征分解 + 特征值下限）。"""
    ev, U = np.linalg.eigh(0.5 * (S + S.T))
    return (U * (1.0 / np.sqrt(np.maximum(ev, floor)))) @ U.T


def b2c_b(E):
    """批量 body 系相对旋转 -> 相机系：R_CL E R_CL^T（V.R_CL 实测 b->c）。"""
    return V.R_CL @ E @ V.R_CL.T


def so3_proj(M):
    """批量极分解投影回 SO(3)（SVD）。

    为什么要它：b2c_b 用的 V.R_CL 正交精度只有 ~1e-6（实测 max|RR^T-I|=9e-7 量级），
    于是相机系区间矩阵 A^T B 的 trace 会**超出 3**（实测 +8.6e-7）。arccos(clip(...))
    把超出部分截成 1 => **凡是 <=0.1 deg 的角度被误报成 0**。本段的地板数字全落在
    0.03~0.15 deg 这个区间里，正是被污染的范围（第一轮没发现，第二轮自检抓到）。
    投影后 trace 误差降到 1e-16，小角度不再塌陷。"""
    U, _, Vt = np.linalg.svd(M)
    U = U.copy()
    U[..., -1] *= np.linalg.det(np.matmul(U, Vt))[:, None]
    return np.matmul(U, Vt)


def ang_pair(A, Bm):
    """批量成对角距（度）：angle(A_i^T B_i)，先投影回 SO(3) 再取 arccos（见 so3_proj）。"""
    return np.degrees(np.arccos(np.clip(
        0.5 * (np.trace(so3_proj(np.matmul(np.swapaxes(A, -1, -2), Bm)),
                       axis1=1, axis2=2) - 1.0), -1.0, 1.0)))


def med(x):
    x = np.asarray(x, float); x = x[np.isfinite(x)]
    return float(np.median(x)) if len(x) else np.nan


def p90(x):
    x = np.asarray(x, float); x = x[np.isfinite(x)]
    return float(np.percentile(x, 90)) if len(x) else np.nan


def imu_traj(ti, gi):
    """IMU 角速度梯形积分 -> Traj（平移置零）。陀螺在 body 系 => 必须**右乘**递推。"""
    Rs = np.empty((len(ti), 3, 3))
    Rs[0] = np.eye(3)
    dt = np.diff(ti)
    for k in range(len(dt)):
        Rs[k + 1] = Rs[k] @ V.exp_so3(0.5 * (gi[k] + gi[k + 1]) * dt[k])
    return V.Traj(ti, np.zeros((len(ti), 3)),
                  np.array([V.rot_to_quat_w(r) for r in Rs]))


# --------------------------------------------------------------------------- #
# 阶段一：视觉观测（与扰动无关，只做一次 + 缓存）
# --------------------------------------------------------------------------- #
def measure(K0, Wn, cache, verbose=True):
    t0 = time.time()
    if os.path.exists(cache):
        z = np.load(cache)
        m = {k: z[k] for k in z.files}
        if verbose:
            print('阶段一 命中测量缓存 %s：%d 帧 / %d 因子 —— 不重测图像'
                  % (cache, len(m['Rclean']), len(m['Rvis'])))
        return m

    tl, _, _ = B.read_lidar_stamps(stride=1)
    tum = np.loadtxt(BASE)
    lio = V.Traj(tum[:, 0], tum[:, 1:4], tum[:, 4:8])
    ti, gi, _ = B.read_imu()
    imu = imu_traj(ti, gi)
    te = tl[K0:K0 + Wn] + DT0                    # 曝光中点时刻
    Pp, Rp, vp = lio.at(te)                       # 参照：连续轨迹 slerp 插值
    _, Ri, vi = imu.at(te)

    frames = B.read_images(range(K0, K0 + Wn))    # 只读需要的帧号
    gray = {k: V.prep_gray(frames[k][1]) for k in frames}
    del frames
    t_img = time.time() - t0

    Rvis = np.tile(np.eye(3), (Wn - 1, 1, 1))
    Sig = np.zeros((Wn - 1, 3, 3))
    usable = np.zeros(Wn - 1, bool)
    tcn = np.full(Wn - 1, np.nan)
    thv = np.full(Wn - 1, np.nan)
    sgv = np.full(Wn - 1, np.nan)
    ninl = np.zeros(Wn - 1, int)
    why = []
    t_tr = t_est = 0.0
    for i in range(Wn - 1):
        k = K0 + i
        a0 = time.time()
        a, b, _ = V.track_pair(gray[k], gray[k + 1])
        t_tr += time.time() - a0
        if a is None or len(a) < 40:
            why.append('track_fail')
            continue
        _, tc, tn = RT.cam_rel_from_lio(lio, te[i], te[i + 1])
        if tn is None or tn < MINBASE:
            why.append('baseline_too_short')
            tcn[i] = 0.0 if tn is None else tn
            continue
        a0 = time.time()
        d = RT.estimate_rotation_d(a, b, tc)
        t_est += time.time() - a0
        tcn[i] = tn
        why.append('ok' if d['usable'] else str(d.get('reason', 'fail'))[:18])
        if not d['usable']:
            continue
        Rvis[i] = d['R']
        Sig[i] = 0.5 * (d['cov'] + d['cov'].T)
        usable[i] = True
        thv[i] = d['th_deg']
        sgv[i] = d['sigma_max_deg']
        ninl[i] = d['n_inl']

    np.savez(cache, idx=np.arange(K0, K0 + Wn), te=te, Rclean=Rp, Pclean=Pp,
             vpose=vp, Rvis=Rvis, Sig=Sig, usable=usable, tcn=tcn, thv=thv,
             sgv=sgv, ninl=ninl, Rimu=Ri, RimuValid=vi)
    if verbose:
        cnt = {}
        for w in why:
            if w != 'ok':
                cnt[w] = cnt.get(w, 0) + 1
        print('阶段一 测量 Wn=%d 帧 (k=%d..%d, %.1f s)：可用视觉因子 %d/%d = %.1f%%'
              % (Wn, K0, K0 + Wn - 1, te[-1] - te[0], int(usable.sum()), Wn - 1,
                 100.0 * usable.mean()))
        print('         读图+去畸变 %.1f s | LK 跟踪 %.1f s | 深度辅助求解 %.1f s'
              ' | 合计 %.1f s' % (t_img, t_tr, t_est, time.time() - t0))
        print('         |tc| med=%.4f m (p25 %.4f) | 区间旋转 med=%.3f deg | '
              '自报 sigma_max med=%.4f deg'
              % (med(tcn), np.nanpercentile(tcn, 25), med(thv[usable]),
                 med(sgv[usable])))
        print('         LIO 插值失效 %d/%d 帧 | 内点 med %d | 失败 %s'
              % (int((~vp).sum()), Wn, med(ninl[usable]), str(cnt)))
    return dict(idx=np.arange(K0, K0 + Wn), te=te, Rclean=Rp, Pclean=Pp, vpose=vp,
                Rvis=Rvis, Sig=Sig, usable=usable, tcn=tcn, thv=thv, sgv=sgv,
                ninl=ninl, Rimu=Ri, RimuValid=vi)


# --------------------------------------------------------------------------- #
# 阶段二：注入已知扰动（真值 delta）
# --------------------------------------------------------------------------- #
def make_delta(level, N, te):
    """delta[N,3]：世界系旋转向量（弧度）。注入式 R_pert = Exp(delta) R_clean。"""
    rng = np.random.default_rng(SEED + LEVEL_SEED[level])
    if level == 'L0-zero':
        return np.zeros((N, 3)), {}
    if level == 'L1-iso0.05':
        A = 0.05 * D2R
        d = rng.standard_normal((N, 3))
        return d * (A / np.linalg.norm(d, axis=1, keepdims=True)), {}
    if level == 'L2-rand0.10':
        return rng.standard_normal((N, 3)) * (0.10 * D2R / np.sqrt(3.0)), {}
    if level == 'L3-alt0.20':
        A = 0.20 * D2R
        axis = rng.standard_normal(3)
        axis /= np.linalg.norm(axis)
        sg = np.where(np.arange(N) % 2 == 0, 1.0, -1.0)
        return sg[:, None] * A * axis[None, :], {}
    A_s, f_s, A_n, phi = 0.5 * D2R, 0.2, 0.10 * D2R, 0.7
    d = np.zeros((N, 3))
    d[:, 2] = A_s * np.sin(2 * np.pi * f_s * (te - te[0]) + phi)
    d += rng.standard_normal((N, 3)) * (A_n / np.sqrt(3.0))
    return d, dict(amp=A_s, freq=f_s, phi=phi, noise=A_n,
                   dt=float(te[1] - te[0]))


# --------------------------------------------------------------------------- #
# 修正器：纯旋转因子图 + 向量化 LM + S1/S2 约束
# --------------------------------------------------------------------------- #
class Problem:
    """min sum ||Cw r_vis||^2 + sum ||dtheta||^2/sig_p^2
              + sum ||dtheta_k - 2 dtheta_{k-1} + dtheta_{k-2}||^2 / sig_d2^2

    r_vis,i = log( R_vis,i^T b2c(R'_{i+1}^T R'_i) )，R'_k = Exp(u_k) Rp_k。
    fidx 是**本问题数组内**（=窗口内）的位姿/因子下标；sig_p / sig_d2 以 deg 传入。
    解析 Jacobian（左乘参数化：u_i -> u_i + d 使 R'_i -> (I+[d]x)R'_i）。推导要点：
    扰动只落在区间旋转的右侧 B_new = B Exp(eta)，eta = R_CL R'_i^T d_world（左位姿取 +、
    右位姿取 -），再经 log(Exp(r)Exp(eta)) ~ r + Jl(r)^{-1} eta，最后把世界系左乘扰动换回
    加法参数 d_world = Jl(u)^T d_u：
        dr/du_i     =  Jl(r)^{-1} R_CL R'_i^T Jl(u_i)^T
        dr/du_{i+1} = -Jl(r)^{-1} R_CL R'_i^T Jl(u_{i+1})^T
    三个方向因子脚本启动时都用中心差分核实（写反会立刻被自检抓住）。
    """

    def __init__(self, Rp, Rvis, Cw, sig_p, sig_d2, fidx):
        self.Rp = np.asarray(Rp)
        self.n = len(self.Rp)
        self.fidx = np.asarray(fidx, int)
        self.m = len(self.fidx)
        self.RvisT = np.swapaxes(np.asarray(Rvis)[self.fidx], -1, -2)
        self.Cw = np.asarray(Cw)[self.fidx]
        self.Cp = np.eye(3) / (sig_p * D2R)
        self.Cd = None if sig_d2 is None else np.eye(3) / (sig_d2 * D2R)
        self.fw = np.ones(self.m)
        self.nr_v = 3 * self.m
        self.nr_p = 3 * self.n
        self.nr_d = 0 if self.Cd is None else 3 * max(0, self.n - 2)
        self.nres = self.nr_v + self.nr_p + self.nr_d
        self.nvar = 3 * self.n
        self._templates()

    def _templates(self):
        a3 = np.arange(3)
        m, n, f = self.m, self.n, self.fidx
        s = np.arange(m)
        # 视觉块：每因子 3 行 x（左位姿 3 列 + 右位姿 3 列）
        self.v_row = np.repeat(3 * s[:, None] + a3[None, :], 3)          # (3m,)
        cb = np.broadcast_to(3 * f[:, None, None] + a3[None, None, :],
                            (m, 3, 3)).ravel()
        self.v_c1, self.v_c2 = cb, cb + 3
        # 一阶先验块：每行 3 列
        self.p_row = np.repeat(self.nr_v + 3 * np.arange(n)[:, None]
                               + a3[None, :], 3)
        self.p_col = np.broadcast_to(3 * np.arange(n)[:, None, None]
                                     + a3[None, None, :], (n, 3, 3)).ravel()
        self.p_val = np.tile(self.Cp, (n, 1)).ravel()
        # 二阶差分块：残差行 i-2，列 i / i-1 / i-2
        self.d_row = None
        if self.Cd is not None and n >= 3:
            i = np.arange(2, n)
            self.d_row = np.repeat(self.nr_v + self.nr_p + 3 * (i - 2)[:, None]
                                   + a3[None, :], 3)
            base = np.broadcast_to(3 * i[:, None, None] + a3[None, None, :],
                                   (len(i), 3, 3)).ravel()
            self.d_cols = (base, base - 3, base - 6)
            self.d_val = np.tile(self.Cd, (len(i), 1)).ravel()

    def value(self, u, want_jac=True):
        """返回 (res, (rows, cols, vals), chi)；vals 用行主序展平以匹配 rows/cols。"""
        ui = u.reshape(self.n, 3)
        Ri = exp_b(ui) @ self.Rp
        f = self.fidx
        E = np.matmul(np.swapaxes(Ri[f + 1], -1, -2), Ri[f])       # body 系区间旋转
        r = log_b(np.matmul(self.RvisT, b2c_b(E)))                # (m,3) rad
        w = np.einsum('mij,mj->mi', self.Cw, r)
        chi = np.linalg.norm(w, axis=1)
        res = np.zeros(self.nres)
        res[:self.nr_v] = (self.fw[:, None] * w).ravel()
        res[self.nr_v:self.nr_v + self.nr_p] = (ui @ self.Cp.T).ravel()
        if self.d_row is not None:
            d2 = ui[2:] - 2 * ui[1:-1] + ui[:-2]
            res[self.nr_v + self.nr_p:] = (d2 @ self.Cd.T).ravel()
        if not want_jac:
            return res, None, chi
        # A_new = A Exp(eta)，eta = +/- R_CL R'_i^T d_world；
        # d log(A Exp(eta))/d eta|_0 = Jl(r)^-1；加法扰动 d_u 与左乘 d_world 之间还差
        # Exp(d_world)Exp(u) = Exp(u + Jl(u)^-T d_world) => d_world = Jl(u)^T d_u。
        # 三步都由启动时的中心差分自检核实（写错方向立刻暴露）。
        K = np.matmul(np.linalg.inv(jl_b(r)), np.matmul(
            V.R_CL, np.swapaxes(Ri[f], -1, -2)))
        JlU = np.swapaxes(jl_b(ui), -1, -2)
        M = self.fw[:, None, None] * np.matmul(self.Cw, K)
        Mi = np.matmul(M, JlU[f])
        Mj = -np.matmul(M, JlU[f + 1])
        rows = [self.v_row, self.v_row, self.p_row]
        cols = [self.v_c1, self.v_c2, self.p_col]
        vals = [Mi.ravel(), Mj.ravel(), self.p_val]
        if self.d_row is not None:
            for cc, sg in zip(self.d_cols, (1.0, -2.0, 1.0)):
                rows.append(self.d_row)
                cols.append(cc)
                vals.append(sg * self.d_val)
        return res, (np.concatenate(rows), np.concatenate(cols),
                     np.concatenate(vals)), chi

    def _lm(self, u, iters, tol, sparse):
        if self.m == 0 and self.d_row is None:
            return np.zeros(self.nvar)          # 只剩先验 => 解为 0
        lam = 1e-4
        r, trip, _ = self.value(u)
        cost = 0.5 * float(r @ r)
        for _ in range(iters):
            rows, cols, vals = trip
            if sparse:
                Js = sp.coo_matrix((vals, (rows, cols)),
                                   shape=(self.nres, self.nvar)).tocsr()
                H = (Js.T @ Js).tocsc()
                g = Js.T @ r
                hd = np.asarray(H.diagonal()).ravel()
            else:
                J = np.zeros((self.nres, self.nvar))
                J[rows, cols] = vals
                H = J.T @ J
                g = J.T @ r
                hd = np.diag(H).copy()
            ok = False
            for _t in range(7):
                if sparse:
                    A = (H + sp.diags(lam * np.maximum(hd, 1e-12) + 1e-12)).tocsc()
                    try:
                        d = spla.spsolve(A, -g)
                    except Exception:                              # noqa: BLE001
                        lam *= 10.0
                        continue
                else:
                    A = H + np.diag(lam * np.maximum(hd, 1e-12) + 1e-12)
                    try:
                        d = np.linalg.solve(A, -g)
                    except np.linalg.LinAlgError:
                        lam *= 10.0
                        continue
                if not np.all(np.isfinite(d)):
                    lam *= 10.0
                    continue
                rn = self.value(u + d, False)[0]
                cn = 0.5 * float(rn @ rn)
                if cn < cost:
                    u = u + d
                    cost = cn
                    lam = max(lam * 0.25, 1e-12)
                    ok = True
                    break
                lam *= 8.0
            if not ok or np.linalg.norm(d) < tol:
                break
            r, trip, _ = self.value(u)
        return u

    def solve(self, u0, iters=8, tol=1e-7, sparse=False, irls=1):
        u = np.array(u0, float).ravel()
        for p in range(irls):
            u = self._lm(u, iters, tol, sparse)
            if p + 1 < irls and self.m:
                chi = self.value(u, False)[2]
                self.fw = np.minimum(1.0, HUBER_C / np.maximum(chi, 1e-9))
        return u.reshape(self.n, 3)


def apply_bounds(u, clip_deg, prev=None):
    """S1 绝对钳位 ||dtheta_k|| <= clip，S2 旋转 slew ||dtheta_k - dtheta_{k-1}|| <= slew。

    slew 限制的对象是**修正量的一阶差分**，不是修正后的角速度、也不是角加速度。
    clip 限的是"这一帧往回收多少"，不保证帧间连续（相邻两帧可以朝相反方向各收回
    clip）—— 这正是 L3（符号交替）档专门要检验的问题，故配 slew = 2*clip 同步收紧。
    实现：解完后按序投影回可行域（硬投影，注释里说明它是软实现的等价形式）。
    prev：在线滑窗传入"上一帧已发布"的 dtheta，使首元素的 slew 相对真实历史判定；
          批处理留 None（整段数组内部自行连乘投影）。
    """
    clip = clip_deg * D2R
    slew = SLEW_FACTOR * clip
    out = np.array(u, float)
    n_clip = n_slew = 0
    last = None if prev is None else np.asarray(prev, float)
    for k in range(len(out)):
        nv = float(np.linalg.norm(out[k]))
        if nv > clip:
            out[k] *= clip / nv
            n_clip += 1
        ref = last if k == 0 else out[k - 1]
        if ref is not None:
            d = out[k] - ref
            nd = float(np.linalg.norm(d))
            if nd > slew:
                out[k] = ref + d * (slew / nd)
                n_slew += 1
        last = out[k]
    return out, dict(clip=n_clip, slew=n_slew)


def run_slide(Rp, Rvis, Cw, sig_p, sig_d2, usable, clip_deg, iters=6):
    """在线因果滑窗：逐帧解窗口 [k-W+1..k]，只取最新帧 dtheta_k，零额外延迟。

    热启动按**全局位姿号**映射上一窗的解（投影后的值写回历史，保证下一窗的热启动
    同样满足钳位/slew）。
    """
    n = len(Rp)
    hist = np.zeros((n, 3))
    n_clip = n_slew = 0
    for k in range(n):
        lo = max(0, k - W_SLIDE + 1)
        loc = np.arange(lo, k)[usable[lo:k]] - lo       # 窗口内局部因子下标
        if loc.size == 0:
            sol = np.zeros((k - lo + 1, 3))
        else:
            pr = Problem(Rp[lo:k + 1], Rvis[lo:k], Cw[lo:k], sig_p, sig_d2, loc)
            u0 = hist[lo:k + 1].copy()
            sol = pr.solve(u0, iters=iters, sparse=False, irls=1)
        v, act = apply_bounds(sol[-1:], clip_deg,
                              prev=None if k == 0 else hist[k - 1])
        hist[k] = v[0]
        n_clip += act['clip']
        n_slew += act['slew']
    return hist, dict(clip=n_clip, slew=n_slew)


def run_batch(Rp, Rvis, Cw, sig_p, sig_d2, usable, clip_deg):
    """整段稀疏批处理：全局最优上界（非因果、零相位），延迟口径与滑窗不可横比。"""
    n = len(Rp)
    loc = np.arange(n - 1)[usable]
    if loc.size == 0:
        return np.zeros((n, 3)), dict(clip=0, slew=0)
    pr = Problem(Rp, Rvis, Cw, sig_p, sig_d2, loc)
    sol = pr.solve(np.zeros((n, 3)), iters=25, sparse=True, irls=2)
    out, act = apply_bounds(sol, clip_deg)
    return out, act


# --------------------------------------------------------------------------- #
# 阶段五：纯平滑对照（完全不接图像）
# --------------------------------------------------------------------------- #
def smooth_rel(Rseq, mode, alpha=None, half=None):
    """对 body 系区间旋转向量滤波后重新积分回姿态（平移仍用 LIO）。

    mode='ema'    ：因果指数平滑，在线口径，群延迟 (1-alpha)/alpha 帧；
    mode='center' ：中心窗，**离线零相位但固定延迟 half 帧**（不是因果）。
    三种口径（离线零相位 / 在线因果 / 固定延迟）不得横比，故返回 delay 供标注。
    """
    n = len(Rseq)
    w = log_b(np.matmul(np.swapaxes(Rseq[1:], -1, -2), Rseq[:-1]))
    ws = np.zeros_like(w)
    if mode == 'ema':
        acc = w[0].copy()
        for i in range(n - 1):
            acc = alpha * w[i] + (1.0 - alpha) * acc
            ws[i] = acc
        delay = (1.0 - alpha) / alpha
    else:
        for i in range(n - 1):
            a0, b0 = max(0, i - half), min(n - 1, i + half + 1)
            ws[i] = w[a0:b0].mean(axis=0)
        delay = float(half)
    out = np.zeros((n, 3, 3))
    out[0] = Rseq[0]
    for i in range(n - 1):
        # w_i = log(R_{i+1}^T R_i) 是 b_{i+1}<-b_i，重构要右乘其逆（转置）：
        # R_{i+1} = R_i · Exp(w_i)^T  —— 少转置等于把真实转角取反，会累积成度级漂移。
        out[i + 1] = V.rotm(out[i] @ V.exp_so3(ws[i]).T)
    return out, delay


# --------------------------------------------------------------------------- #
# 指标
# --------------------------------------------------------------------------- #
def build_traj(Rclean, delta, u):
    """R_pert = Exp(delta) R_clean ；R' = Exp(u) R_pert。

    不人为钉住首帧：绝对朝向本就不可观，由一阶先验软定标，残余共模如实计入 cm 列。
    """
    Rp = np.matmul(exp_b(delta), Rclean)
    return Rp, np.matmul(exp_b(u), Rp)


def hf3(Rseq):
    """旋转版 3-tap：hf[k] = 0.5||w_k - w_{k-1}||，w_k = log(R_k^T R_{k+1})（度）。"""
    w = log_b(np.matmul(np.swapaxes(Rseq[1:], -1, -2), Rseq[:-1])) * RAD
    return 0.5 * np.linalg.norm(np.diff(w, axis=0), axis=1)


def detrend_common(v):
    """交付②的去共模：**逐轴**减均值 + 减一次（线性）趋势项。
    目的是剥掉"图像通道与 IMU 通道共享的系统性分量"（残余外参/时间戳偏置、陀螺常值偏
    差等），只留下**区间级**（逐帧涨落）的改善；常值/线性分量不该记作图像修正的功劳。
    注意这与"位姿 dtheta 的共模"是两回事：后者在区间里恒等抵消（见 add_cm 的自检）。"""
    v = np.asarray(v, float)
    n = len(v)
    r = v - v.mean(axis=0)
    if n > 2:
        t = np.arange(n) - (n - 1) / 2.0
        r = r - np.outer(t, (t @ r) / float(t @ t))
    return r


def image_chain(Rclean, Rvis, usable):
    """表 0：纯图像链（不做任何优化）——即"先验权重->0、视觉因子当硬约束"的极限解，
    用来给修正器定"原则上可达"的天花板。锚定：首帧朝向取 Rclean[0]（绝对朝向本就不可观；
    跨 N 帧的**相对**累积误差不依赖该锚定）。body 系区间 E=R_CL^T R_vis R_CL，
    递推 R_chain[k+1]=R_chain[k] E^T；不可用因子处退回 LIO 区间（统计时另行只用全可用窗口）。"""
    n = len(Rclean)
    Eb = np.matmul(np.swapaxes(V.R_CL, -1, -2), np.matmul(Rvis, V.R_CL))
    El = np.matmul(np.swapaxes(Rclean[1:], -1, -2), Rclean[:-1])
    E = np.where(usable[:, None, None], Eb, El)
    out = np.empty((n, 3, 3))
    out[0] = Rclean[0]
    for k in range(n - 1):
        out[k + 1] = V.rotm(out[k] @ E[k].T)
    return out


def span_err(Ra, Rb, nstep, cu):
    """跨 nstep 帧累积相对旋转误差(deg)：angle(Ra[k+n]^T Ra[k], Rb[k+n]^T Rb[k])。
    cu 为"不可用区间"的前缀和，只统计窗口内区间全可用的 k，避免混入 LIO 填补值。"""
    idx = np.arange(len(Ra) - nstep)
    if nstep > 0:
        idx = idx[(cu[idx + nstep] - cu[idx]) == 0]
    if len(idx) == 0:
        return np.array([np.nan]), 0
    Pa = np.matmul(np.swapaxes(Ra[idx + nstep], -1, -2), Ra[idx])
    Pb = np.matmul(np.swapaxes(Rb[idx + nstep], -1, -2), Rb[idx])
    return ang_pair(Pa, Pb), len(idx)


def smooth_sg(Rseq, win, causal=False, poly=2):
    """交付④：局部多项式(Savitzky-Golay 型)平滑**区间旋转向量**，作为"无稳态延迟"的
    纯平滑基线。poly=2 => 对任意二次变化的区间旋转序列**稳态误差为零**（EMA/中心移动
    平均做不到：它们的群延迟会把真实运动误记成失真，第一版就是这么让判决门廉价通过的）。
      causal=False：窗口关于中心对称 => 离线零相位口径（边缘对称收缩，延迟 0）
      causal=True ：只用过去 win 帧 => 固定延迟口径（群延迟 (win-1)/2 帧）
    与 smooth_rel 一样只看轨迹自身 => 共模完全不动，这才是"纯平滑"的定义。"""
    n = len(Rseq)
    w = log_b(np.matmul(np.swapaxes(Rseq[1:], -1, -2), Rseq[:-1]))
    ws = np.zeros_like(w)
    for i in range(n - 1):
        if causal:
            lo, hi = max(0, i - win + 1), i + 1
        else:
            h = win // 2
            lo, hi = max(0, i - h), min(n - 1, i + h + 1)
        seg = w[lo:hi]
        m = len(seg)
        if m <= poly:
            ws[i] = seg.mean(axis=0)
            continue
        x = (np.arange(-(m - 1), 1.0) if causal else np.arange(m) - m // 2.0)
        X = np.vander(x, poly + 1, increasing=True)
        ws[i] = np.linalg.lstsq(X, seg, rcond=None)[0][0]   # 评估点 x=0 的常值项
    out = np.empty((n, 3, 3))
    out[0] = Rseq[0]
    for i in range(n - 1):
        out[i + 1] = V.rotm(out[i] @ V.exp_so3(ws[i]).T)
    return out


def add_cm(Rseq, c):
    """把常值共模旋转 Exp(c) **精确左乘**到每个位姿：R''=Exp(c)R'。
    区间 R''_{k+1}^T R''_k = R'_{k+1}^T R'_k 逐点恒等 => 任何"区间级"指标必须纹丝不动，
    而位姿级指标（res / cm）会变。用它实测检验"vs IMU 被共模白嫖"这一怀疑的真伪。"""
    return np.matmul(exp_b(np.tile(np.asarray(c, float), (len(Rseq), 1))), Rseq)


def metrics(Rc, Rclean, delta, Rimu_c, floor_vl):
    e = log_b(np.matmul(Rc, np.swapaxes(Rclean, -1, -2))) * RAD   # 残余扰动（度）
    e_n = np.linalg.norm(e, axis=1)
    d = np.asarray(delta) * RAD
    e_rms = float(np.sqrt(np.mean(np.sum(e ** 2, axis=1))))
    d_rms = float(np.sqrt(np.mean(np.sum(d ** 2, axis=1))))
    db = np.nan if d_rms < 1e-9 else 10.0 * np.log10(
        float(np.mean(np.sum(d ** 2, axis=1)))
        / max(float(np.mean(np.sum(e ** 2, axis=1))), 1e-24))
    dd, de = np.diff(d, axis=0), np.diff(e, axis=0)
    v_d = float(np.mean(np.sum(dd ** 2, axis=1)))
    v_e = float(np.mean(np.sum(de ** 2, axis=1)))
    db_d = np.nan if v_d < 1e-18 else 10.0 * np.log10(v_d / max(v_e, 1e-24))
    out = dict(e=e, d_rms=d_rms, e_rms=e_rms, db=db, db_d=db_d,
               res_med=med(e_n), res_p90=p90(e_n),
               cm=float(np.linalg.norm(np.mean(e, axis=0))),
               e_int=med(hf3(Rc)), e_int90=p90(hf3(Rc)), floor=floor_vl)
    Rc_c = b2c_b(np.matmul(np.swapaxes(Rc[1:], -1, -2), Rc[:-1]))
    Rcl_c = b2c_b(np.matmul(np.swapaxes(Rclean[1:], -1, -2), Rclean[:-1]))
    out['int_med'] = med(ang_pair(Rc_c, Rcl_c))     # 区间残差 vs R_clean（有地板）
    out['int_p90'] = p90(ang_pair(Rc_c, Rcl_c))
    out['imu_base'] = med(ang_pair(Rcl_c, Rimu_c))  # 未修正 LIO vs IMU
    out['imu_med'] = med(ang_pair(Rc_c, Rimu_c))    # 修正后 vs IMU
    out['imu_p90'] = p90(ang_pair(Rc_c, Rimu_c))
    # 交付②：vs IMU 的**去共模**口径（区间误差向量逐轴减均值 + 减一次趋势后再取模长
    # 中位数）。基线与修正后用同一变换，可直接横比；imu_cm 是被扣掉的常值分量，透明化。
    g_corr = log_b(np.matmul(np.swapaxes(Rimu_c, -1, -2), Rc_c)) * RAD
    g_base = log_b(np.matmul(np.swapaxes(Rimu_c, -1, -2), Rcl_c)) * RAD
    out['imu_dt'] = med(np.linalg.norm(detrend_common(g_corr), axis=1))
    out['imu_dt_base'] = med(np.linalg.norm(detrend_common(g_base), axis=1))
    out['imu_cm'] = float(np.linalg.norm(np.mean(g_corr, axis=0)))
    out['Rc_c'] = Rc_c
    return out


def sine_retention(e, info):
    """L4：残余扰动的世界 Z 分量对 0.2 Hz 做锁相拟合 -> 幅值保留率 % 与相关系数。

    保留率 100% = 真实低频运动完整保住；明显 <100% = 被当噪声抹掉（判不通过）。
    """
    if 'amp' not in info:
        return None
    tt = np.arange(len(e)) * info['dt']
    ph = 2.0 * np.pi * info['freq'] * tt + info['phi']
    s, c = np.sin(ph), np.cos(ph)
    z = e[:, 2]
    Xd = np.stack([s, c, np.ones_like(s)], axis=1)
    cf = np.linalg.lstsq(Xd, z, rcond=None)[0]
    amp = float(np.hypot(cf[0], cf[1]))
    # res = Z 轴剥掉真实正弦后的残余（=该轴的噪声水平）；nz = 三轴合成的残余噪声模长，
    # 判 L4 档"能否把 0.1 deg 白噪声压到 <=0.05" 只能用它，不能用 res_med（含真实正弦）。
    zf = z - Xd[:, :2] @ cf[:2]
    nn = np.linalg.norm(np.c_[e[:, 0], e[:, 1], zf], axis=1)
    # e 已是**度**（metrics 里乘过 RAD），故拟合幅值直接就是度，不再乘 RAD。
    return dict(pct=100.0 * amp / (info['amp'] * RAD),
                cor=float(np.corrcoef(z, s)[0, 1]),
                amp_fit=amp,
                res=float(np.median(np.abs(zf))),
                nz=float(np.median(nn)), nz90=float(np.percentile(nn, 90)))


def jac_selfcheck(n=5):
    """解析 Jacobian vs 中心差分（相对误差必须 < 1e-3，否则后面数字全部不可用）。"""
    rng = np.random.default_rng(7)
    Rp = np.stack([V.rotm(V.exp_so3(rng.standard_normal(3) * 0.02))
                   for _ in range(n)])
    Rvis = np.stack([V.rotm(V.exp_so3(rng.standard_normal(3) * 0.03))
                     for _ in range(n - 1)])
    Cw = np.stack([np.eye(3) * (300.0 + 40 * i) for i in range(n - 1)])
    pr = Problem(Rp, Rvis, Cw, 0.1, 0.25, np.arange(n - 1))
    u = rng.standard_normal((n, 3)) * 0.003
    _, trip, _ = pr.value(u.ravel())
    J = np.zeros((pr.nres, pr.nvar))
    J[trip[0], trip[1]] = trip[2]
    Jd = np.zeros_like(J)
    h = 1e-6
    for j in range(pr.nvar):
        up = u.copy(); up[j // 3, j % 3] += h
        um = u.copy(); um[j // 3, j % 3] -= h
        Jd[:, j] = (pr.value(up.ravel(), False)[0]
                    - pr.value(um.ravel(), False)[0]) / (2 * h)
    err = float(np.abs(J - Jd).max())
    return err, err / max(float(np.abs(Jd).max()), 1e-12)


# --------------------------------------------------------------------------- #
# 单配置求解（模块级纯函数：不打印、不动 rows，以便 fork 到子进程并行跑）
# --------------------------------------------------------------------------- #
_G = {}          # 只读上下文；main() 在**建池之前**填好，子进程 fork 后直接继承


def solve_cfg(mode, wk, level, sp_, sd, clip, base, it=None):
    """一个配置（在线滑窗/整批 × 加权口径 × 注入档 × 先验 × 钳位 × 基线轨迹）-> 指标。"""
    G = _G
    n = G['N']
    delta, info = G['LEV'][level]
    Rp, _ = build_traj(base, delta, np.zeros((n, 3)))
    Cw = G['CW'][wk]
    if mode == 'slide':
        u, act = run_slide(Rp, G['Rvis'], Cw, sp_, sd, G['usable'], clip,
                           iters=G['IT'] if it is None else it)
    else:
        u, act = run_batch(Rp, G['Rvis'], Cw, sp_, sd, G['usable'], clip)
    _, Rc = build_traj(base, delta, u)
    mt = metrics(Rc, base, delta, G['Ri_c'], G['floor_vl'])
    mt.update(level=level, mode=mode, W=wk, sp=sp_,
              sd='off' if sd is None else sd, clip=clip, u=u, act=act,
              base=base, Rpose=Rc, delay=0.0 if mode == 'slide' else np.nan,
              ret=sine_retention(mt['e'], info))
    return mt


def _job(args):
    return solve_cfg(*args)


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #
def main():
    flags = [a for a in sys.argv[1:] if a.startswith('--')]
    pos = [int(a) for a in sys.argv[1:] if not a.startswith('--')]
    Wn = pos[0] if pos else 350
    K0 = pos[1] if len(pos) > 1 else 500
    smoke = '--smoke' in flags
    cache = '/tmp/p0f_meas_%d_%d.npz' % (Wn, K0)
    t_all = time.time()

    print('=' * LINES)
    print('P0-F 旋转扰动注入 -> 滑窗+纯旋转因子图恢复力   K0=%d Wn=%d seed=%d '
          'W_slide=%d%s' % (K0, Wn, SEED, W_SLIDE, '   [SMOKE 缩网格]' if smoke else ''))
    print('=' * LINES)

    e0, r0 = jac_selfcheck()
    print('[自检] 解析 Jacobian vs 中心差分 max|diff|=%.2e（相对 %.2e）' % (e0, r0))
    assert r0 < 1e-3, '解析 Jacobian 不自洽，后续数字全部不可用'

    m = measure(K0, Wn, cache)
    Rclean, te, usable = m['Rclean'], m['te'], m['usable']
    Rvis, Rimu, N = m['Rvis'], m['Rimu'], len(m['Rclean'])

    # ---------------- 地板 / kappa 标定（全部实测） ----------------
    Rcl_c = b2c_b(np.matmul(np.swapaxes(Rclean[1:], -1, -2), Rclean[:-1]))
    Ri_c = b2c_b(np.matmul(np.swapaxes(Rimu[1:], -1, -2), Rimu[:-1]))
    ok_imu = m['RimuValid'][:-1] & m['RimuValid'][1:]
    d_vl = np.where(usable, ang_pair(Rvis, Rcl_c), np.nan)     # 图像 vs LIO
    d_vi = np.where(usable & ok_imu, ang_pair(Rvis, Ri_c), np.nan)
    d_li = np.where(ok_imu, ang_pair(Rcl_c, Ri_c), np.nan)
    tr_sig = np.array([np.trace(0.5 * (S + S.T)) if u else np.nan
                       for S, u in zip(m['Sig'], usable)])
    kappa = float(np.sqrt(np.nanmean(d_vl ** 2 * D2R ** 2
                                     / np.maximum(tr_sig, 1e-20))))
    # 另一口径：以 IMU 为参照（剥掉 LIO 自身误差）=> 只反映视觉通道本身的散布。
    kappa_imu = float(np.sqrt(np.nanmean(d_vi ** 2 * D2R ** 2
                                         / np.maximum(tr_sig, 1e-20))))
    floor = (SIG_FLOOR * D2R) ** 2
    Ahat = np.zeros((N - 1, 3, 3))          # 自报协方差的逆平方根（未膨胀）
    for i in range(N - 1):
        if usable[i]:
            Ahat[i] = inv_sqrt_sym(m['Sig'][i], floor)
    # 交付③第三种口径：直接用作本段**实测**的视觉本征误差协方差（相机区间系，
    # 各向同性化前先按逐因子残差样本合成），完全丢弃自报协方差的形状信息。
    g_vi = np.zeros((N - 1, 3))
    for i in range(N - 1):
        if usable[i] and ok_imu[i]:
            # log_b 已是**弧度**，与 m['Sig']（rad^2）同单位，不再乘 D2R（乘了会把协方差
            # 缩小 3283 倍、全部落到特征值下限，W3 就退化成 sigma=SIG_FLOOR 而非实测值）。
            g_vi[i] = log_b(np.matmul(np.swapaxes(Rvis[i:i + 1], -1, -2),
                                       Ri_c[i:i + 1]))[0]
    gs = g_vi[np.linalg.norm(g_vi, axis=1) > 0]
    Sig_vi = (gs.T @ gs) / max(len(gs), 1)
    Aindep = np.tile(inv_sqrt_sym(Sig_vi, floor), (N - 1, 1, 1))
    # 三种因子噪声口径（残差 r=Log(V_k^T*R'_区间) 的权重 Cw=Sigma^-1/2）：
    CW = dict()
    CW['W1 kappa_vl'] = Ahat / kappa                 # 第一版标称（把基线误差当噪声）
    CW['W2 kappa_vi'] = Ahat / kappa_imu             # 视觉本征 + 逐因子形状
    CW['W3 sigma_vi'] = Aindep                       # 视觉本征 + 各向同性（不信自报形状）
    WKEYS = ('W1 kappa_vl', 'W2 kappa_vi', 'W3 sigma_vi')
    # Cw 的逆特征值就是该因子各主轴的 sigma（rad），用来把三种口径折算成"等效 sigma"。
    sigma_deg = {}
    for k, v in CW.items():
        sg = 1.0 / np.linalg.eigvalsh(v[usable]) * RAD
        sigma_deg[k] = float(np.median(sg.mean(axis=1)))
    floor_vl = med(d_vl)

    print('\n[地板 / 标定]（口径同 §36.2，均为本段实测）')
    print('   图像 vs LIO  med=%.4f deg p90=%.4f deg  <- "区间残差 vs R_clean" 的不可约地板'
          % (med(d_vl), p90(d_vl)))
    print('   图像 vs IMU  med=%.4f deg p90=%.4f deg  <- 视觉通道本身能达到的最好水平'
          % (med(d_vi), p90(d_vi)))
    print('   LIO  vs IMU  med=%.4f deg p90=%.4f deg  <- 基线自身区间误差'
          % (med(d_li), p90(d_li)))
    print('   自报 sigma_max med=%.4f deg -> chi2 膨胀 kappa=%.1f / kappa_imu=%.1f'
          '（Sigma_vis = kappa^2 Sigma_hat）' % (med(m['sgv'][usable]), kappa, kappa_imu))
    print('   实测视觉本征协方差 Sig_vi 主轴 sigma = %s deg（特征值下限 %.3f deg，'
          '低于下限的轴被抬到下限）'
          % (np.array2string(np.sqrt(np.maximum(np.linalg.eigvalsh(Sig_vi), floor)) * RAD,
                             precision=4, floatmode='fixed'), SIG_FLOOR))
    print('   三种因子噪声口径（Cw=Sigma^-1/2，等效 sigma 主轴均值中位数）：')
    for wk in WKEYS:
        print('     %-12s sigma=%.4f deg  %s' % (wk, sigma_deg[wk], {
            WKEYS[0]: '自报协方差 x 以"图像 vs LIO"标定的 kappa —— **第一版标称**，'
                      '把待消除的基线误差当成噪声（本轮要检验的伪影源）',
            WKEYS[1]: '自报协方差 x 以"图像 vs IMU"标定的 kappa —— 视觉本征，保留逐因子'
                      '协方差形状（交付③主口径）',
            WKEYS[2]: '直接用作本段实测"图像 vs IMU"误差协方差，丢弃自报形状（鲁棒性检验）'}[wk]))
    print('   推导：残差 r=Log(V_k^T R\'_区间) 对未知量 dtheta 而言噪声项**只有视觉自身误差**；'
          '基线区间误差 (a_{k+1}-a_k) 属于**待求信号**，不是噪声。')
    print('   修正器输入：usable 因子 %d/%d，|tc| med=%.4f m' %
          (int(usable.sum()), N - 1, med(m['tcn'][usable])))

    grid_p = MAIN_SP if not smoke else (0.30,)
    grid_d2 = MAIN_SD if not smoke else (None,)
    fine_sp = GRID_SIG_P if not smoke else (0.30,)
    fine_d2 = GRID_SIG_D2 if not smoke else (None,)
    fine_c = FINE_CLIP if not smoke else (0.30,)
    IT = 4 if not smoke else 6          # 滑窗 LM 迭代数（自检里验证 3/4/6 的一致性）

    def uniq(seq):
        """去重保序（None 与数值可混排；smoke 下网格退化后避免重复配置）。"""
        out = []
        for x in seq:
            if not any((x is y) or (x is not None and y is not None and x == y)
                       for y in out):
                out.append(x)
        return tuple(out)

    # 主表固定 clip：smoke 只给一档时退化到唯一档，避免越界。
    CLIP_MAIN = fine_c[min(1, len(fine_c) - 1)]
    # 运行预算（<=5 min）：实测整段 N=118 时滑窗**单配置串行 ~2.4 s**，全网格串行要
    # 超 ~6 min 预算。配置之间彼此独立、路径上无随机性（delta 由固定子种子预生成），
    # 故用 fork 进程池并行求解 => 网格**不裁**：表 A 四档全跑、表 A2 clip 全档、
    # 表 E/E2 两个 sp 端都保留。并行只改调度，不改任何数字（子集串行复算可比对）。
    A_LEVELS = LEVELS[1:]
    A2_SP = uniq(MAIN_SP) if not smoke else (0.30,)
    A2_SD = (0.50, None)
    A2_CLIP = fine_c

    rows = []
    LEV = {}
    for level in LEVELS:
        delta, info = make_delta(level, N, te)
        LEV[level] = (delta, info)

    EXTRA_CW = dict(CW)
    EXTRA_CW['W4 kappa=1'] = Ahat       # 极限口径：完全信任自报 sigma（等价"硬拉向图像"）

    def d_rms_of(level):
        """该档注入扰动的 RMS（度），直接从真值 delta 算，不依赖 make_delta 的 info。"""
        return float(np.sqrt(np.mean(np.sum((LEV[level][0] * RAD) ** 2, axis=1))))

    def one(mode, wk, level, sp, sd, clip, Rb=None, it=None):
        """跑一个配置 -> 指标 -> 记入 rows。Rb=None 以 LIO(R_clean) 为参照/扰动基，
        否则以传入的"干净基线"为参照（表 E/E2 的良定义口径）。"""
        mt = solve_cfg(mode, wk, level, sp, sd, clip,
                       Rclean if Rb is None else Rb, it=it)
        rows.append(mt)
        return mt

    # 上下文交给模块级 _G，**必须在建池之前**（fork 后子进程只读继承）。
    _G.clear()
    _G.update(N=N, LEV=LEV, CW=EXTRA_CW, Rvis=Rvis, usable=usable,
              Ri_c=Ri_c, floor_vl=floor_vl, IT=IT)
    NP_ = max(1, min(12, mp.cpu_count()))
    if '--serial' in flags:                    # 仅用于验证"并行不改数字"
        NP_ = 1
    try:
        POOL = mp.get_context('fork').Pool(NP_) if NP_ > 1 else None   # fork：子进程直接继承 _G
    except Exception:                       # noqa: BLE001  池建不起来就退回串行
        POOL = None
    print('\n[并行] 进程池 %d worker（配置间独立，结果与串行一致）'
          % (NP_ if POOL is not None else 1))

    def par(jobs):
        """一批配置并行求解，**按提交顺序**返回并登记进 rows。"""
        if not jobs:
            return []
        if POOL is None or len(jobs) == 1:
            res = [solve_cfg(*j) for j in jobs]
        else:
            res = list(POOL.imap(_job, jobs, chunksize=1))
        for mt in res:
            rows.append(mt)
        return res

    def prow(mt):
        r = mt['ret']
        pre = ((mt['W'] + ' ' + mt['level']) if mt['mode'] == 'batch' else mt['W'])
        print('    %-26s sp=%-5.2f sd=%-4s cl=%.2f | dB=%s d=%s | res=%.4f/%.4f'
              ' cm=%.4f | int=%.4f' %
              (pre, mt['sp'], mt['sd'], mt['clip'], fmt(mt['db']), fmt(mt['db_d']),
               mt['res_med'], mt['res_p90'], mt['cm'], mt['int_med']))
        print('               imu %.4f->%.4f (去共模 %.4f->%.4f, 扣掉常值 %.4f)'
              ' | S1=%d S2=%d%s' %
              (mt['imu_base'], mt['imu_med'], mt['imu_dt_base'], mt['imu_dt'],
               mt['imu_cm'], mt['act']['clip'], mt['act']['slew'],
              '' if r is None else ' | ret=%.1f%% cor=%.3f nz=%.4f'
              % (r['pct'], r['cor'], r['nz'])))

    # ================= 表 0：纯图像链基线（交付①，决定性） =================
    print('\n' + '=' * LINES)
    print('表 0 纯图像链基线（交付①）：相邻帧视觉相对旋转**直接积分**，不做任何优化')
    print('=' * LINES)
    Rch = image_chain(Rclean, Rvis, usable)
    ok_int = usable.copy()                    # 区间两端都要：视觉可用 & IMU 有效
    ok_int[:N - 1] &= m['RimuValid'][:N - 1] & m['RimuValid'][1:N]
    bad_cum = np.concatenate(([0.0], np.cumsum(~ok_int)))
    print('   锚定：链首帧取 R_clean[0]（绝对朝向本就不可观）；下表是**跨 N 帧累积相对**'
          '误差，与锚定无关。')
    print('   可用区间 %d/%d（IMU 位姿有效性失效 %d）；只统计窗口内区间全可用的 k'
          % (int(ok_int.sum()), N - 1, int((~ok_int & usable).sum())))
    print('   %5s %8s %14s %10s %12s %10s %10s'
          % ('跨帧N', '样本数', '图像链 med', '链 p90', 'LIO med', 'LIO p90', '§36.2记录'))
    ch_span = {}
    for ns in SPANS:
        e_ch, n_s = span_err(Rch, m['Rimu'], ns, bad_cum)
        e_li, _ = span_err(Rclean, m['Rimu'], ns, bad_cum)
        ch_span[ns] = (med(e_ch), med(e_li))
        print('   %5d %8d %14.4f %10.4f %12.4f %10.4f %10s'
              % (ns, n_s, med(e_ch), p90(e_ch), med(e_li), p90(e_li),
                 ('%.3f' % REF_362[ns]) if ns in REF_362 else '-'))
    ch1 = ch_span[1][0]
    print('   N=1 是构造恒等（链的区间就是 R_vis，=%.4f deg），它的含义是：**修正器若完全'
          '信任图像，区间误差就会落到这个值** => 这就是区间级天花板。' % ch1)
    print('   判读：链的**区间**误差 %.4f deg << LIO %.4f deg，说明"把区间拉到图像"原则上'
          '可达；但链的**累积**误差随 N 快速发散（N=20 %.3f deg），说明绝对/低频朝向'
          '不可用，与 §36.2 记录（N=2 %.3f -> N=20 %.3f）同量级。'
          % (ch1, ch_span[1][1], ch_span[20][0], REF_362[2], REF_362[20]))
    # 链轨迹的完整指标（含不可用区间的 LIO 填补），作为"硬拉向图像"的参照行
    mt = metrics(Rch, Rclean, np.zeros((N, 3)), Ri_c, floor_vl)
    mt.update(level='L0-zero', mode='chain', W='图像链', sp=np.nan, sd='-', base=Rclean,
              clip=np.nan, u=None, act=dict(clip=0, slew=0), delay=np.nan, ret=None,
              Rpose=Rch, d_rms=0.0)
    rows.append(mt)
    prow(mt)

    # ================= 表 A：三口径加权 × 在线因果滑窗（交付③） =================
    print('\n' + '=' * LINES)
    print('表 A 在线因果滑窗（W=%d，零额外延迟，clip 固定 %.2f deg，细扫见表 A2）'
          % (W_SLIDE, CLIP_MAIN))
    print('   dB=抑制比(位姿域) d=抑制比(一阶差分域) res=残余扰动 vs 参照 int=区间残差 vs 参照'
          ' imu=修正后区间 vs IMU（含常值）去共模=剥掉常值+线性趋势后')
    print('=' * LINES)
    t_scan = time.time()
    n_cfg = 0
    for wk in WKEYS:
        print('\n---- 因子噪声口径 %s（等效 sigma=%.4f deg）----' % (wk, sigma_deg[wk]))
        for level in A_LEVELS:
            print('   %s：注入 RMS %.4f deg' % (level, d_rms_of(level)))
            jb = [('slide', wk, level, sig_p, sig_d2, CLIP_MAIN, Rclean, None)
                  for sig_p in grid_p for sig_d2 in grid_d2]
            for mt in par(jb):
                prow(mt)
                n_cfg += 1
        print('     （累计滑窗 %d 配置墙钟 %.1f s，平均 %.3f s/配置）'
              % (n_cfg, time.time() - t_scan, (time.time() - t_scan) / n_cfg))

    # ================= 表 A2：决定性口径上的 clip / 先验 / lambda2 细扫 =================
    print('\n' + '=' * LINES)
    print('表 A2 细扫（口径 %s；补 clip>=0.5/1.0 与 lambda2=off，sp/sd/clip 全网格，'
          '只在 L2/L3 两档）' % WKEYS[1])
    print('=' * LINES)
    for level in ('L2-rand0.10', 'L3-alt0.20'):
        print('   %s' % level)
        jb = [('slide', WKEYS[1], level, sig_p, sig_d2, clip, Rclean, None)
              for sig_p in A2_SP for sig_d2 in A2_SD for clip in A2_CLIP
              if not (sig_p in grid_p and sig_d2 in grid_d2
                      and abs(clip - CLIP_MAIN) < 1e-9)]      # 已在表 A
        for mt in par(jb):
            prow(mt)
            n_cfg += 1
    print('   极限口径 W4（完全信任自报 sigma，等价"硬拉向图像"，sp=1.00 sd=off cl=%.2f）'
          '：应与表 0 的图像链一致' % fine_c[-1])
    jb = [('slide', 'W4 kappa=1', level, 1.00, None, fine_c[-1], Rclean, None)
          for level in ('L2-rand0.10', 'L3-alt0.20')]
    for mt in par(jb):
        prow(mt)
        n_cfg += 1
    print('   收敛性自检（同一配置不同 LM 迭代数，验 iters=%d 够不够）：' % IT)
    its = (3, IT, 6, 8)
    jb = [('slide', WKEYS[1], 'L2-rand0.10', 1.00, None, fine_c[-1], Rclean, it)
          for it in its]
    for it, mt in zip(its, par(jb)):
        print('     iters=%d -> dB=%s res=%.5f imu=%.5f 去共模=%.5f'
              % (it, fmt(mt['db']), mt['res_med'], mt['imu_med'], mt['imu_dt']))
        n_cfg += 1

    # ================= 表 A3：L0 零扰动"误修量"（三口径对照） =================
    print('\n' + '=' * LINES)
    print('表 A3 L0 零扰动档：修正器在没有东西可修时把轨迹挪了多少（误修量）')
    print('=' * LINES)
    jb = [('slide', wk, 'L0-zero', sig_p, None, CLIP_MAIN, Rclean, None)
          for wk in WKEYS for sig_p in grid_p]
    for mt in par(jb):
        prow(mt)
        n_cfg += 1

    # ================= 表 B：整段批处理上界（三口径） =================
    print('\n' + '=' * LINES)
    print('表 B 整段批处理（全局最优上界，非因果 / 零相位，延迟不得与表 A 横比）')
    print('=' * LINES)
    jb = [('batch', wk, level, sig_p, None, clip, Rclean, None)
          for wk in WKEYS for level in LEVELS
          for sig_p in uniq((grid_p[0], grid_p[-1]))
          for clip in uniq((CLIP_MAIN, fine_c[-1]))]
    tb = time.time()
    for mt in par(jb):
        prow(mt)
    print('   （整批 %d 配置墙钟 %.1f s）' % (len(jb), time.time() - tb))

    # ---------------- 阶段五：纯平滑对照（交付④） ----------------
    print('\n' + '=' * LINES)
    print('表 C 纯平滑对照（完全不接图像，只用 LIO 位姿自身；res/dB 相对 LIO 计算，与表 A 同参照）')
    print('   sg-c=局部二次(Savitzky-Golay)因果窗，固定延迟 (w-1)/2 帧，对二次运动**零稳态误差**')
    print('   sg-0=同上但窗关于中心对称，离线零相位（延迟 0，非因果）')
    print('   ema =在线因果指数平滑（自带群延迟；第一版基线，保留作对照）')
    print('   ctr =中心移动平均（离线零相位 + 固定延迟 h 帧）')
    print('   三种延迟口径（在线因果 / 固定延迟 / 离线零相位）**不得横比**，只在同类内比 dB。')
    print('=' * LINES)

    def sm_row(level, mode, tag, Rs, dly, wpar):
        delta, info = LEV[level]
        mt = metrics(Rs, Rclean, delta, Ri_c, floor_vl)
        mt.update(level=level, mode='smooth-' + mode, W=tag, sp=wpar, sd='-',
                  base=Rclean, u=None, clip=np.nan, act=dict(clip=0, slew=0),
                  Rpose=Rs, delay=dly, ret=sine_retention(mt['e'], info))
        rows.append(mt)
        prow(mt)
        return mt

    sg_wins = SG_WINS if not smoke else (5,)
    for level in LEVELS:
        delta, info = LEV[level]
        Rp, _ = build_traj(Rclean, delta, np.zeros((N, 3)))
        b0 = metrics(Rp, Rclean, delta, Ri_c, floor_vl)
        print('\n  ---- %s（注入 RMS %.4f deg；不平滑 res=%.4f imu=%.4f 去共模=%.4f）----'
              % (level, d_rms_of(level), b0['res_med'], b0['imu_med'], b0['imu_dt_base']))
        print('       口径         sp    sd   cl | dB        d      | res/p90    cm    | int')
        for w in sg_wins:
            sm_row(level, 'sg-c', 'sg-c w=%d' % w, smooth_sg(Rp, w, causal=True),
                   (w - 1) / 2.0, float(w))
            sm_row(level, 'sg-0', 'sg-0 w=%d' % w, smooth_sg(Rp, w, causal=False),
                   0.0, float(w))
        for a in ((0.7, 0.5, 0.15) if not smoke else (0.5,)):
            Rs, dly = smooth_rel(Rp, 'ema', alpha=a)
            sm_row(level, 'ema', 'ema a=%.2f' % a, Rs, dly, a)
        for h in ((1, 2, 5) if not smoke else (2,)):
            Rs, dly = smooth_rel(Rp, 'center', half=h)
            sm_row(level, 'ctr', 'ctr h=%d' % h, Rs, dly, float(h))

    # ---------------- 表 D：交付② 去共模 vs IMU 区间误差 ----------------
    print('\n' + '=' * LINES)
    print('表 D 去共模的 vs IMU **区间**误差（交付②）+ "共模能否白嫖该指标"的实测检验')
    print('=' * LINES)

    def base_row(level):
        Rp, R0 = build_traj(Rclean, LEV[level][0], np.zeros((N, 3)))
        return metrics(R0, Rclean, LEV[level][0], Ri_c, floor_vl)

    print('   D.0 不变性自检：把**大**常值共模 Exp(c) 精确左乘到修正后位姿上。代数上区间')
    print('       R\'\'_{k+1}^T R\'\'_k = R\'_{k+1}^T R\'_k 逐点恒等，故所有**区间级**指标')
    print('       （imu_med / 去共模 / int）必须一模一样，只有位姿级 res 会变。')
    ref = None
    pool = [r for r in rows if r['mode'] == 'slide' and r['level'] == 'L2-rand0.10'
            and r['W'] in WKEYS]
    if pool:
        ref = max(pool, key=lambda r: (r['db'] if np.isfinite(r['db']) else -9e9))
        cdir = np.asarray(ref['e'].mean(axis=0)) * D2R       # e 已是度 -> 转弧度
        cn = float(np.linalg.norm(cdir))
        # 刻意放大到 1.00 deg：共模越大，"能否白嫖区间级指标"的检验越有说服力。
        c_rad = cdir / max(cn, 1e-12) * (1.00 * D2R)
        mm = metrics(ref['Rpose'], ref['base'], LEV['L2-rand0.10'][0], Ri_c, floor_vl)
        mc = metrics(add_cm(ref['Rpose'], c_rad), ref['base'],
                     LEV['L2-rand0.10'][0], Ri_c, floor_vl)
        print('       dθ 共模 c=%s deg（模 %.4f）；检验用共模放大到 %.2f deg，方向同 c；'
              '配置 %s sp=%.2f sd=%s cl=%.2f'
              % (np.array2string(ref['e'].mean(axis=0), precision=4), cn * RAD, 1.00,
                 ref['W'], ref['sp'], ref['sd'], ref['clip']))
        print('       %-14s %10s %10s %10s %10s' % ('', 'imu_med', '去共模', 'int', 'res'))
        for nm, r_ in (('未加共模', mm), ('加共模后', mc)):
            print('       %-14s %10.5f %10.5f %10.5f %10.5f'
                  % (nm, r_['imu_med'], r_['imu_dt'], r_['int_med'], r_['res_med']))
        print('       区间级最大变化 = %.2e deg（res 变化 %.4f deg）=> %s'
              % (max(abs(mc['imu_med'] - mm['imu_med']), abs(mc['imu_dt'] - mm['imu_dt']),
                     abs(mc['int_med'] - mm['int_med'])),
                 abs(mc['res_med'] - mm['res_med']),
                 '共模**不能**白嫖区间级指标（怀疑不成立）'
                 if max(abs(mc['imu_med'] - mm['imu_med']), abs(mc['imu_dt'] - mm['imu_dt']),
                        abs(mc['int_med'] - mm['int_med'])) < 1e-6 else '区间级指标被改变（须查）'))

    print('\n   D.1 去共模方式：g_k=Log(R_imu,k^T R_corr,k)（度，逐帧区间误差向量）先**逐轴**')
    print('       减全段均值，再减一次（线性）趋势项，然后取模长中位数 => 只留区间级涨落，')
    print('       残余外参/时间戳偏置/陀螺常值偏置等系统性分量不计入功劳；基线走同一变换。')
    GROUPS = (('在线因果滑窗(0 额外延迟)', lambda r: r['mode'] == 'slide'),
              ('离线整段批处理', lambda r: r['mode'] == 'batch'),
              ('图像链=硬约束极限', lambda r: r['mode'] == 'chain'),
              ('纯平滑 sg-c(因果固定延迟)', lambda r: r['mode'] == 'smooth-sg-c'),
              ('纯平滑 sg-0(离线零相位)', lambda r: r['mode'] == 'smooth-sg-0'),
              ('纯平滑 ema(因果带群延迟)', lambda r: r['mode'] == 'smooth-ema'))
    for level in LEVELS:
        br = base_row(level)
        print('\n  %s：LIO 基线 vs IMU 区间 med=%.4f -> **去共模 med=%.4f** deg'
              % (level, br['imu_base'], br['imu_dt_base']))
        print('   %-26s %-16s %-14s %9s %9s %8s %7s'
              % ('组', '最优配置', '口径', '修正后', '去共模后', '改善比', 'dB'))
        for gname, sel in GROUPS:
            g = [r for r in rows if sel(r) and r['level'] == level]
            if not g:
                continue
            bb = min(g, key=lambda r: r['imu_dt'])
            ratio = bb['imu_dt'] / br['imu_dt_base'] if br['imu_dt_base'] > 0 else np.nan
            print('   %-26s %-16s %-14s %9.4f %9.4f %8.2f %7s'
                  % (gname, 'sp=%.2f sd=%s cl=%.2f' % (bb['sp'], bb['sd'], bb['clip']),
                     bb['W'], bb['imu_med'], bb['imu_dt'], ratio, fmt(bb['db'])))

    # ---------------- 表 E：良定义复测（干净基线注入） ----------------
    print('\n' + '=' * LINES)
    print('表 E 良定义复测：把扰动注入"干净基线"而非 LIO 本身')
    print('=' * LINES)
    print('   为什么必须做：表 A/C 的 res/dB 是相对 R_clean(=LIO) 算的，而 LIO 自带区间误差')
    print('   %.4f deg > 注入的 %.4f deg，于是"把基线真实误差一起修掉"被 res 记成"误修"，'
          % (med(d_li), d_rms_of('L2-rand0.10')))
    print('   该门在 eps(基线误差)>delta(注入) 时**根本不可辨**。表 E 先取 L0 零扰动 + 弱先验')
    print('   (sp=%.2f sd=off cl=%.2f) 的修正结果作"干净基线"R_base（其 vs IMU 区间误差单独'
          % (fine_sp[-1], fine_c[-1]))
    print('   实测、作新的不可约地板），再把同样的 delta 注入 R_base 重跑 => res 只含注入量。')
    mE = one('slide', WKEYS[1], 'L0-zero', fine_sp[-1], None, fine_c[-1])
    mE['tbl'] = 'E'
    Rbase = mE['Rpose']
    print('   R_base：vs IMU 区间 med=%.4f 去共模=%.4f（原 LIO %.4f/%.4f）；res(对 LIO)=%.4f'
          % (mE['imu_med'], mE['imu_dt'], mE['imu_base'], mE['imu_dt_base'], mE['res_med']))
    print('   说明：本表 res 相对 R_base，dB 亦相对 R_base，故可直接与 %.2f deg 目标比。' % TARGET)
    print('   %14s %-12s %-16s %8s %8s %9s %9s' % ('口径', '档位', 'sp/sd/clip', 'dB',
                                                   'res med', 'imu', '去共模'))
    jb = [('slide', wk, level, sp_, None, fine_c[-1], Rbase, None)
          for wk in WKEYS for level in LEVELS[1:4]
          for sp_ in uniq((grid_p[0], grid_p[-1]))]
    for mt in par(jb):
        mt['tbl'] = 'E'
        print('   %14s %-12s sp=%.2f sd=off cl=%.2f %8s %8.4f %9.4f %9.4f'
              % (mt['W'], mt['level'], mt['sp'], fine_c[-1], fmt(mt['db']), mt['res_med'],
                 mt['imu_med'], mt['imu_dt']))
        n_cfg += 1

    # ---- 表 E2：基线换成纯图像链再注入（决定性排除"求解器没在修"）----
    print('\n' + '=' * LINES)
    print('表 E2 基线 = 纯图像链（不做任何优化）+ 已知注入：直接回答"求解器到底在不在修"')
    print('=' * LINES)
    print('   为什么还必须有这张表：表 E 的 R_base 仍是"被图像拉着"的轨迹，它相对图像还留着')
    print('   残余区间误差，于是修正器看到的信号 = 注入 delta **加上**那段残余，res 无法把')
    print('   "修掉 delta" 和"继续往图像拉"分开。图像链的区间**按构造**就等于 R_vis，未扰动时')
    print('   视觉残差 ~0（不可用因子处除外），注入后残差里的唯一信号就是 delta 本身 => 此时')
    print('   res = ||delta - dtheta|| 是真恢复力，可修下限只剩窗口内共模不可观（约 delta_rms/')
    print('   sqrt(W-1)）与估计噪声。')
    print('   锚定同表 0（首帧取 R_clean[0]）；注入方式与表 E 完全相同（同一 delta、同一档）。')
    print('   %14s %-12s %6s %7s %9s %9s %9s %9s %8s'
          % ('加权口径', '档位', 'sp', 'dB', 'res med', 'res p90', 'imu_med', '去共模',
             '可修下限'))
    jb = [('slide', wk, level, sp_, None, fine_c[-1], Rch, None)
          for wk in WKEYS for level in LEVELS[1:4]
          for sp_ in uniq((grid_p[0], grid_p[-1]))]
    for mt in par(jb):
        mt['tbl'] = 'E2'
        print('   %14s %-12s %6.2f %7s %9.4f %9.4f %9.4f %9.4f %8.4f'
              % (mt['W'], mt['level'], mt['sp'], fmt(mt['db']), mt['res_med'],
                 mt['res_p90'], mt['imu_med'], mt['imu_dt'],
                 d_rms_of(mt['level']) / np.sqrt(W_SLIDE - 1.0)))
        n_cfg += 1

    # ---------------- 汇总 + 修订判决（第二轮） ----------------
    print('\n' + '=' * LINES)
    print('汇总与修订判决（第二轮复核）')
    print('=' * LINES)
    mrows = [r for r in rows if not str(r.get('tbl', '')).startswith('E')]
    erows = [r for r in rows if r.get('tbl') == 'E']
    e2rows = [r for r in rows if r.get('tbl') == 'E2']
    DT_OK = 0.80        # 门 2 阈值：去共模后 <= 0.80 x LIO 基线（即改善 >=20%）
    CH1_OK = 0.06       # 门 1 阈值：你（父代理）给定的"可达带"上界（0.03~0.06 deg）
    CORE = ('L1-iso0.05', 'L2-rand0.10')   # 门 2 主判据档（扰动量级 = 待修的真实误差）

    def grp(mode, level, wk=None, pool=None):
        g = [r for r in (mrows if pool is None else pool)
             if r['mode'] == mode and r['level'] == level and (wk is None or r['W'] == wk)]
        return g

    def fmax(vals):
        v = [x for x in vals if x is not None and np.isfinite(x)]
        return max(v) if v else float('nan')

    def fmin(vals):
        v = [x for x in vals if x is not None and np.isfinite(x)]
        return min(v) if v else float('nan')

    print('\n  一览：每档 × 每口径**最优**（按"去共模 vs IMU 区间误差"选）')
    print('   %-13s %-12s %-21s %7s %8s %8s %8s %7s %6s'
          % ('档位', '口径/组', '参数', 'dB', 'res', 'imu', '去共模', '改善比', '延迟'))
    for level in LEVELS:
        br = base_row(level)
        for wk in WKEYS:
            g = grp('slide', level, wk)
            if not g:
                continue
            bb = min(g, key=lambda r: r['imu_dt'])
            print('   %-13s %-12s %-21s %7s %8.4f %8.4f %8.4f %7.2f %6.1f'
                  % (level, wk, 'sp=%s sd=%s cl=%s' % (bb['sp'], bb['sd'], bb['clip']),
                     fmt(bb['db']), bb['res_med'], bb['imu_med'], bb['imu_dt'],
                     bb['imu_dt'] / br['imu_dt_base'], bb['delay']))
        for mode, tag in (('batch', '批处理(离线)'), ('chain', '图像链(硬极限)'),
                          ('smooth-sg-c', 'SG因果'), ('smooth-sg-0', 'SG零相位'),
                          ('smooth-ema', 'EMA因果'), ('smooth-ctr', '中心窗')):
            g = grp(mode, level)
            if not g:
                continue
            bb = min(g, key=lambda r: r['imu_dt'])
            print('   %-13s %-12s %-21s %7s %8.4f %8.4f %8.4f %7.2f %6.1f'
                  % (level, tag, str(bb['W']), fmt(bb['db']), bb['res_med'],
                     bb['imu_med'], bb['imu_dt'], bb['imu_dt'] / br['imu_dt_base'],
                     bb['delay']))
        print('   %-13s %-12s %-21s %7s %8s %8.4f %8.4f %7.2f %6.1f'
              % (level, '基线=LIO', '未修正', '-', '-', br['imu_base'],
                 br['imu_dt_base'], 1.0, 0.0))

    print('\n  [门 1 / 交付①] 区间级天花板（纯图像链，不做任何优化）')
    base_li = med(d_li)
    print('   图像链 N=1（逐帧区间）误差 vs IMU = %.4f deg；LIO 同口径 = %.4f deg；'
          '图像 vs IMU 标定值 = %.4f deg' % (ch1, base_li, med(d_vi)))
    print('   链跨帧累积 vs IMU：' + '  '.join('N=%d:%.3f' % (s, ch_span[s][0]) for s in SPANS))
    print('   LIO 跨帧累积 vs IMU：' + '  '.join('N=%d:%.3f' % (s, ch_span[s][1]) for s in SPANS))
    # 判据按你给的绝对带来定：<=0.06 可达；>=0.13 = 信息上限。同时打印相对基线比值。
    G1 = bool(ch1 <= CH1_OK)
    print('   => 区间级天花板 %.4f deg（= 基线的 %.2f 倍）%s：把区间拉向图像**原则上可达**'
          '（判据 <= %.2f deg）' % (ch1, ch1 / base_li,
                                    '落在你给的可达带内' if G1 else '不在可达带内', CH1_OK)
          if G1 else
          '   => 区间级天花板 %.4f deg（= 基线的 %.2f 倍）**已接近/达到基线水平**：'
          '链式结构里相关误差累积，负结果属真实信息上限（判据 <= %.2f deg）'
          % (ch1, ch1 / base_li, CH1_OK))

    print('\n  [门 2 / 交付②] 去共模后 vs IMU 区间误差（阈值 <= %.2f x 基线；主判据档 %s）'
          % (DT_OK, '/'.join(CORE)))
    G2 = True
    npass = 0
    for level in LEVELS[1:]:
        br = base_row(level)
        g = grp('slide', level)
        if not g:
            continue
        bb = min(g, key=lambda r: r['imu_dt'])
        ratio = bb['imu_dt'] / br['imu_dt_base']
        ok = bool(ratio <= DT_OK)
        npass += ok
        if level in CORE:
            G2 &= ok
        print('   %-12s 基线 %.4f -> 修正后 %.4f（去共模 %.4f -> %.4f，比 %.2f）%s%s'
              % (level, br['imu_base'], bb['imu_med'], br['imu_dt_base'], bb['imu_dt'],
                 ratio, '达标' if ok else '不达标',
                 '' if level in CORE else '（非主判据档，仅供参考）'))
    print('   参考：L0 零扰动档（没有任何注入信号，改善只能来自"把基线自身误差拉向图像"）'
          '去共模 %.4f -> %.4f'
          % (base_row('L0-zero')['imu_dt_base'],
             min(grp('slide', 'L0-zero'), key=lambda r: r['imu_dt'])['imu_dt']))
    print('   => 门 2 %s（全部 %d 档里 %d 档达标；通过与否只由主档 L1/L2 决定）'
          % ('通过：图像修正带来了区间级（非共模）的真实改善' if G2
             else '不通过：主判据档去共模后没有实质区间级改善', len(LEVELS) - 1, npass))

    print('\n  [门 3 / 交付③] 因子噪声口径：第一版"修不动"是加权伪影还是事实')
    G3 = False
    for level in LEVELS[1:]:
        st = {}
        for wk in WKEYS:
            g = grp('slide', level, wk)
            st[wk] = (fmax([r['db'] for r in g]), fmin([r['res_med'] for r in g]),
                      fmin([r['imu_dt'] for r in g]), fmin([r['imu_med'] for r in g]))
        gain = max(st[WKEYS[1]][0] - st[WKEYS[0]][0], st[WKEYS[2]][0] - st[WKEYS[0]][0])
        gain2 = max(st[WKEYS[1]][2] - st[WKEYS[0]][2], st[WKEYS[2]][2] - st[WKEYS[0]][2])
        hit = bool(gain >= 3.0 or gain2 <= -0.20 * base_row(level)['imu_dt_base'])
        G3 |= hit
        print('   %-12s W1 dB=%s res=%.4f | W2 dB=%s res=%.4f | W3 dB=%s res=%.4f'
              % (level, fmt(st[WKEYS[0]][0]), st[WKEYS[0]][1], fmt(st[WKEYS[1]][0]),
                 st[WKEYS[1]][1], fmt(st[WKEYS[2]][0]), st[WKEYS[2]][1]))
        print('               去共模 W1=%.4f W2=%.4f W3=%.4f -> ΔdB(相对W1)=%+.1f，'
              'Δ去共模=%+.4f deg => %s'
              % (st[WKEYS[0]][2], st[WKEYS[1]][2], st[WKEYS[2]][2], gain, gain2,
                 '换口径后修正器明显动起来' if hit else '换口径没有实质区别'))
    print('   => 门 3 %s' % ('坐实：伪影（原 κ 口径把待消除信号当噪声，压死了修正量）'
                            if G3 else '否证：口径不是决定因素（父代理第①条不成立）'))

    print('\n  [门 4 / 交付④] 分叉判据：不接图像的纯平滑能不能顶替图像修正器')
    print('   核心指标用**去共模 vs IMU 区间误差**（交付②），不用 dB：dB 的参照是 LIO，')
    print('   而平滑/修正都在"偏离 LIO"，dB 对平滑天然不公平（平滑 dB 为负=偏离 LIO 更远，')
    print('   并不代表它 vs 现实更差）。平滑 dB 只作参考一并打印。')
    G4 = True
    SM_ALT = False
    sm_best_all = []
    for level in LEVELS[1:]:
        cd = grp('slide', level)
        sg = grp('smooth-sg-c', level) + grp('smooth-ema', level)
        sg0 = grp('smooth-sg-0', level) + grp('smooth-ctr', level)
        ba = grp('batch', level)
        sm = sg + sg0
        if not (cd and sm):
            continue
        brdt = base_row(level)['imu_dt_base']
        dbc, dsg = fmax([r['db'] for r in cd]), fmax([r['db'] for r in sg])
        i_dbc = fmin([r['imu_dt'] for r in cd])
        i_sm = fmin([r['imu_dt'] for r in sm])
        sm_best_all.append((level, i_sm, brdt))
        # 本门只回答"纯平滑能否顶替图像"；修正器自身是否达标由门 2 负责，不在此混判。
        hit = bool(np.isfinite(i_dbc) and np.isfinite(i_sm) and i_dbc <= i_sm)
        G4 &= hit
        sm_ok = bool(np.isfinite(i_sm) and i_sm <= DT_OK * brdt)
        SM_ALT |= sm_ok
        print('   %-12s 去共模：基线 %.4f | 修正器 %.4f | 纯平滑最优 %.4f（%s）=> %s'
              % (level, brdt, i_dbc, i_sm,
                 min(sm, key=lambda r: r['imu_dt'])['W'],
                 '修正器更强' if i_dbc < i_sm else '平滑不弱于修正器'))
        print('               参考 dB：滑窗 %s vs SG因果/EMA %s（%s）；离线：批处理 %s vs '
              'SG零相位/中心窗 %s'
              % (fmt(dbc), fmt(dsg),
                 '%+.1f dB' % (dbc - dsg) if np.isfinite(dbc - dsg) else 'n/a',
                 fmt(fmax([r['db'] for r in ba]) if ba else float('nan')),
                 fmt(fmax([r['db'] for r in sg0]) if sg0 else float('nan'))))
        print('               平滑自身是否达标（去共模 <= %.2f x 基线）：%s'
              % (DT_OK, '是' if sm_ok else '否'))
    print('   => 门 4 %s' % ('通过：各档位修正器在核心指标（去共模 vs IMU）上都不劣于纯平滑'
                            if G4 else '不通过：存在纯平滑不弱于修正器的档位'))
    if sm_best_all:
        worst = min(r[1] / r[2] for r in sm_best_all)
        print('   => 分叉结论：纯平滑**%s**顶替图像修正器（各档平滑最优去共模/基线 最小值 '
              '= %.2f，判据 <= %.2f）%s'
              % ('可以' if SM_ALT else '不能', worst, DT_OK,
                 '' if SM_ALT else '；即"连图像都不要、直接接平滑"这条路在本段实测中'
                 '不会让区间误差 vs IMU 变好，Phase 0 不因它而降级'))

    print('\n  [门 5 / 良定义复测（表 E，干净基线上注入）]')
    G5 = True
    for level in LEVELS[1:4]:
        g = [r for r in erows if r['level'] == level]
        if not g:
            continue
        bb = min(g, key=lambda r: r['res_med'])
        hit = bool(bb['res_med'] <= TARGET)
        G5 &= hit
        print('   %-12s 相对**干净基线**的最优 res=%.4f（%s sp=%s sd=%s cl=%.2f dB=%s）'
              '，其去共模 %.4f -> %.4f => %s'
              % (level, bb['res_med'], bb['W'], bb['sp'], bb['sd'], bb['clip'],
                 fmt(bb['db']), bb['imu_dt_base'], bb['imu_dt'],
                 '达标' if hit else '不达标'))
    print('   （注：表 E 的基线 R_base 本身仍有 %.4f deg 的不可约 vs IMU 区间误差，'
          'res<=目标只证明"扰动可恢复"）' % mE['imu_dt'])
    print('   => 门 5 %s' % ('通过：扰动恢复力良定义下达标' if G5 else '不通过'))

    print('\n  [门 5b / 表 E2：基线=纯图像链再注入，残差里只有注入的 delta]')
    print('   这是排除"求解器根本没在修"的最干净一问：图像链的区间**按构造**等于 R_vis，')
    print('   未扰动时视觉残差 ~0，故 res = ||delta-dtheta|| 不掺任何"该往图像拉"的真实信号。')
    G5b = True
    for level in LEVELS[1:4]:
        g = [r for r in e2rows if r['level'] == level]
        if not g:
            continue
        bb = min(g, key=lambda r: r['res_med'])
        hit = bool(bb['res_med'] <= TARGET or bb['db'] >= 6.0)
        G5b &= hit
        print('   %-12s 最优 %s sp=%.2f sd=%s cl=%.2f：res med=%.4f p90=%.4f dB=%s'
              '（注入 RMS %.4f，共模不可观下限 %.4f）；去共模 %.4f -> %.4f => %s'
              % (level, bb['W'], bb['sp'], bb['sd'], bb['clip'], bb['res_med'],
                 bb['res_p90'], fmt(bb['db']), d_rms_of(level),
                 d_rms_of(level) / np.sqrt(W_SLIDE - 1.0), bb['imu_dt_base'],
                 bb['imu_dt'], '在修' if hit else '仍修不动'))
    print('   => 门 5b %s' % ('通过：零真实信号基线上已知扰动被真实抑制，求解器没睡' if G5b
                             else '不通过：连零真实信号基线都修不动 => 求解器/信息真有问题'))
    print('   （读法：本表"去共模"两端的参照是**图像链**，不是 LIO，它只说明修正器在链')
    print('    上又叠加了什么方向的分量；恢复力一律以 res/dB 为准，别把这两列混着读。）')

    print('\n  [门 6 / 真实运动保真度 L4 + L0 误修量 + 高频旁证]')
    l4 = [r for r in rows if r['mode'] == 'slide' and r['level'] == 'L4-sine+noise'
          and r.get('tbl') != 'E']
    l4ret = [r for r in l4 if r['ret'] is not None and r['ret']['pct'] >= RET_MIN]
    l4ok = bool(l4ret)
    if l4ret:
        b4 = max(l4ret, key=lambda r: (r['db'] if np.isfinite(r['db']) else -9e9))
        print('    保留率>= %.0f%% 的滑窗配置：%d/%d；其中最优 %s sp=%s sd=%s clip=%.2f'
              ' dB=%s ret=%.1f%% cor=%.3f（剥离正弦后残余噪声 nz=%.4f deg%s）'
              % (RET_MIN, len(l4ret), len(l4), b4['W'], b4['sp'], b4['sd'], b4['clip'],
                 fmt(b4['db']), b4['ret']['pct'], b4['ret']['cor'], b4['ret']['nz'],
                 '，达标' if b4['ret']['nz'] <= TARGET
                 else '，>目标 %.2f => L4 档噪声抑制不达标' % TARGET))
    else:
        b4 = max(l4, key=lambda r: (r['ret'] or {'pct': -1})['pct'])
        rr = b4['ret'] or dict(pct=float('nan'), nz=float('nan'),
                               amp_fit=float('nan'), cor=float('nan'))
        print('    无任何滑窗配置把 0.2 Hz 真实正弦的幅值保留率做到 >= %.0f%%；最高 %.1f%%'
              '（%s sp=%s sd=%s clip=%.2f dB=%s，拟合幅值 %.4f deg、cor=%.3f、'
              '剥离正弦后 nz=%.4f deg）=> 判"把真实运动当噪声抹掉"'
              % (RET_MIN, rr['pct'], b4['W'], b4['sp'], b4['sd'], b4['clip'],
                 fmt(b4['db']), rr['amp_fit'], rr['cor'], rr['nz']))
    for wk in WKEYS:
        l0 = grp('slide', 'L0-zero', wk)
        if l0:
            b0 = min(l0, key=lambda r: r['imu_dt'])
            print('    L0 误修量 %-12s res med=%.4f p90=%.4f cm=%.4f（sp=%s sd=%s '
                  'clip=%.2f）-> 去共模 %.4f（基线 %.4f）'
                  % (wk, b0['res_med'], b0['res_p90'], b0['cm'], b0['sp'], b0['sd'],
                     b0['clip'], b0['imu_dt'], b0['imu_dt_base']))
    print('    3-tap 高频抖动 hf_theta（§36.3 实测其中约 95% 是真实曲率，只作旁证）：')
    for level in ('L2-rand0.10', 'L3-alt0.20', 'L4-sine+noise'):
        Rp_un, _ = build_traj(Rclean, LEV[level][0], np.zeros((N, 3)))
        cd = [r for r in rows if r['mode'] == 'slide' and r['level'] == level]
        if cd:
            bc = min(cd, key=lambda r: r['imu_dt'])
            print('      %s：未修正 med=%.4f -> 修正后 med=%.4f p90=%.4f deg（%s）'
                  % (level, med(hf3(Rp_un)), bc['e_int'], bc['e_int90'], bc['W']))

    PASS_OLD = all(any(r['res_med'] <= TARGET for r in mrows
                       if r['mode'] == 'slide' and r['level'] == lv)
                   for lv in ('L1-iso0.05', 'L2-rand0.10', 'L3-alt0.20'))
    print('\n[判决]')
    print('   第一版原门（res 相对 LIO，三档能否 <= %.2f deg）：%s —— 该门在基线误差 '
          '%.4f > 注入 %.4f 时**不可辨**，只作记录，良定义版本是门 5。'
          % (TARGET, '能' if PASS_OLD else '不能', base_li, d_rms_of('L2-rand0.10')))
    print('   门 1 区间级天花板远低于基线：%s   门 2 去共模有真实改善：%s   '
          '门 3 口径伪影坐实：%s' % (G1, G2, G3))
    print('   门 4 优于/不劣于纯平滑且自身达标：%s（纯平滑可替代？%s）门 5 良定义恢复力'
          '达标：%s   门 6 L4 真实运动保住(>=%.0f%%)：%s'
          % (G4, SM_ALT, G5, RET_MIN, l4ok))
    if not G3:
        VERDICT = ('(c) 父代理的分析不成立：把 κ 换成视觉本征误差（W2/W3）后修正器的'
                   '抑制比/去共模误差并没有实质变化，"修不动"不是加权口径造成的伪影')
    elif G2 and G5b and l4ok and G4:
        VERDICT = ('(a) 第一版判决作废：负结果是因子噪声口径（κ 用"图像 vs LIO"）+ 先验'
                   '支配的伪影；按 W2/W3 口径修正器确实把区间拉向图像，可达配置见下')
    elif not G2 and not G5b:
        VERDICT = ('(b) 口径修正后依旧修不动：区间级改善不存在，属真实信息上限/结构性'
                   '不可观，第一版"不通过"的结论方向正确、根因写错')
    else:
        VERDICT = ('(混合) 换口径后修正器确实动起来（门 3 通过），但改善的**落地价值**'
                   '只在部分门上成立（门 2=%s / 门 5b=%s / 门 4=%s / 门 6=%s）'
                   % (G2, G5b, G4, l4ok))
    print('   => 修订后的 P0-F 归因：%s' % VERDICT)
    print('   （归因判据用门 5b/表 E2 而**不是**门 5/表 E：表 E 的基线 R_base 仍被图像'
          '拉着，其 res 里混着"继续往图像拉"的真实信号，在 eps>delta 时同样不可辨；'
          '只有基线=纯图像链时 res 才只含注入量。）')
    GO = bool(G1 and G2 and G3 and G4 and G5b and l4ok)
    GO_STRICT = bool(GO and G5)
    print('   => P0-F %s，Phase 1（写 C++ 包）%s'
          % ('通过' if GO else '不通过', '可以开' if GO else '不能开'))
    print('   （若把更严格的门 5（表 E，res<= %.2f）也计入：%s）'
          % (TARGET, '同样通过' if GO_STRICT else '不通过，唯一不达标项是门 5'))
    if SM_ALT:
        print('   分叉提示（门 4）：有档位纯平滑自身就能把去共模区间误差压到判据以内 => '
              '"地图层直接接平滑位姿、连图像都不用"是**可行替代**，须与图像方案并列比选。')
    else:
        print('   分叉关闭（门 4）：所有档位上纯平滑自身的去共模区间误差都**不优于不平滑**'
              '（详见门 4 各行），"连图像都不要、直接接平滑"不等价，Phase 0 不因它降级。')

    print('\n  [可达配置（按去共模 vs IMU 最优，在线因果滑窗）]')
    for level in LEVELS:
        g = grp('slide', level)
        if not g:
            continue
        bb = min(g, key=lambda r: r['imu_dt'])
        print('   %-13s %s sp=%s sd=%s clip=%.2f | dB=%s res=%.4f | imu %.4f -> %.4f'
              '（去共模 %.4f -> %.4f）| S1=%d S2=%d%s'
              % (level, bb['W'], bb['sp'], bb['sd'], bb['clip'], fmt(bb['db']),
                 bb['res_med'], bb['imu_base'], bb['imu_med'], bb['imu_dt_base'],
                 bb['imu_dt'], bb['act']['clip'], bb['act']['slew'],
                 '' if bb['ret'] is None else ' | ret=%.1f%% cor=%.3f nz=%.4f'
                 % (bb['ret']['pct'], bb['ret']['cor'], bb['ret']['nz'])))

    print('\n[口径声明（第二轮）/ 仍需你拍板的地方]')
    print('   0) **本轮发现并修掉的数值缺陷（会影响你引用的三个地板数字）**：ang_pair 用')
    print('      arccos(clip(0.5*(trace-1)))，而 V.R_CL 的正交精度只有 ~1e-6，b2c_b 后区间')
    print('      矩阵乘积的 trace 会**超出 3**（实测 +8.6e-7）=> 被 clip 截成 1，凡是 <=0.1')
    print('      deg 的角度被系统性**误报为 0**。你引用的 图像 vs IMU 0.0283 / LIO vs IMU')
    print('      0.1299 / 图像 vs LIO 0.1409 都落在这个被污染的范围里，本轮改为"先 SVD 极')
    print('      分解投影回 SO(3) 再取 arccos"（so3_proj），三个地板必须按本轮实测值重新取。')
    print('   1) 你对第①条的判断成立：κ 用"图像 vs LIO"(=%.1f) 标定确实把待消除的信号'
          '当噪声；换成视觉本征(κ_imu=%.1f / 实测 sigma_vi) 后修正器"敢不敢动"的差别'
          '由门 3 给出。' % (kappa, kappa_imu))
    print('   2) 你对第②条的说法我**不认同**，并给出实测反证（表 D.0）：把常值共模精确')
    print('      左乘到修正后位姿上，区间 R\'\'^T R\'\' 与 R\'^T R\' **逐点恒等**，故')
    print('      imu_med / 去共模 / int 三个**区间级**指标变化为 0，只有位姿级 res 会变。')
    print('      第一版"零扰动下 0.1299->0.0623"是**位姿级**角度（区间被整体转动后与 IMU')
    print('      位姿系的夹角），不是区间级改善，不能被"白嫖"。本轮真正剥掉的是两通道')
    print('      共享的系统性分量（逐轴减均值 + 减一次趋势），基线走同一变换。')
    print('   3) 表 0 的 N=1 是**构造恒等**（链的区间就是 R_vis），它只说明"完全信任图像')
    print('      时区间误差=视觉本征水平 %.4f deg"；链的跨帧累积在 N=%d 已到 %.3f deg，'
          % (ch1, SPANS[-2], ch_span[SPANS[-2]][0]))
    print('      所以天花板只对**区间级**成立，绝对/低频朝向仍不可用（与 §36.2 一致）。')
    print('   4) 去共模的"窗口/阶数"是我自定的（全段一次性减均值+减一次趋势，N-1=%d 个'
          '区间）。若改成滑窗内去共模，数字会变；这一口径需要你确认。' % (N - 1))
    print('   5) 表 E 的"干净基线"R_base 由 W2 弱先验修正得到，本身还带 %.4f deg 去共模'
          % mE['imu_dt'])
    print('      vs IMU 区间误差（不是真值），故表 E 的 res<=0.05 只证明**恢复力**，不证明')
    print('      绝对精度；真正的绝对精度参照只有 IMU/外部真值。')
    print('   6) 门 4 的等延迟并不严格：滑窗是 0 额外延迟，SG 因果窗有 (w-1)/2=%.1f 帧'
          % ((sg_wins[-1] - 1) / 2.0))
    print('      固定延迟（但对二次运动无稳态失真）。若要严格等延迟，应把滑窗输出也推后')
    print('      同样帧数——本轮未做，只保证"不再用带群延迟的 EMA 当唯一基线"。')
    print('   7) 运行预算：滑窗 LM iters=%d（表 A2 末给出 3/%d/6/8 收敛对照，用于确认'
          '迭代数未截断）；网格不裁，改为配置级 fork 进程池并行（配置彼此独立、无随机性，'
          % (IT, IT))
    print('      与串行逐位一致），故表 A/A2/A3/B/E/E2 都是全网格。')
    print('      Huber/IRLS 仍只在批处理里生效；S1/S2 是解后硬投影（非严格约束优化）。')
    print('   8) sigma 特征值下限 %.3f deg、kappa 取标量、L4 正弦绕世界 Z 轴、tc 取自未'
          % SIG_FLOOR)
    print('      扰动 LIO 等口径与第一版相同，未改。')

    if POOL is not None:
        POOL.close()
        POOL.join()
    print('\n总耗时 %.1f s（滑窗扫参墙钟 %.1f s，%d 滑窗/整批配置，%d worker 并行）'
          % (time.time() - t_all, time.time() - t_scan, n_cfg,
             NP_ if POOL is not None else 1))
    if '--keep' not in flags:
        try:
            os.remove(cache)
            print('已删除测量缓存 %s' % cache)
        except OSError:
            pass


def fmt(x):
    return 'n/a' if x is None or not np.isfinite(x) else '%+6.2f' % x


if __name__ == '__main__':
    main()
