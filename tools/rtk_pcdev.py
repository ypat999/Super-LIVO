#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三层"点云偏差 vs 距离"评测（RTK-SLAM Dataset / construction_seq2）。

参考真值 = 官方 FAST-LIVO2 的稠密建图（本数据集中唯一有稠密建图输出、且图像
真正参与建图的方法）。在其上按体素拟合局部平面，得到一张"参考面网"，然后
把每个被测系统的雷达点，用**该系统自己的 ENU 位姿**搬到参考面上量偏差。

三层各答一件事，互不重叠（避免同一个误差被数三遍）：

L1 距离分层平面散布 σ(range)
   每个参考面先减掉自身均值（= 去掉该面的系统性偏置，也顺带去掉全局配准偏移），
   只留"围绕该面的散布"。散布随距离上升的斜率 = 角度类误差（位姿抖动/测向误差），
   截距 = 测距噪声 + 单帧 footprint。按 σ²(R)=σ0²+(σθ·R)² 拟合，直接给 σ0[cm] 与
   σθ[°]。另给"层间距" layer_std：同一面内各帧均值的散布 = 纯重影/分层。

L2 跨帧几何一致性（含真·图像重投影）
   同一物理面被相隔 Δt 的两帧各测一次，用被测系统自己的位姿把帧 i 的点送到帧 j：
     · point-to-plane：沿参考面法向的错位。最近邻搜索会在切向吸附，但对法向不
       敏感，所以这个量真正反映相对位姿误差。
     · 重投影残差：把预测点投进帧 j 的**虚拟相机**（内参 fx/fy/cx/cy 来自数据集
       标定，外参 Rcl/Pcl 来自 calib.yaml），在图像平面上找帧 j 实测的最近像素点，
       给像素位移 [px] 与同一条视线上的深度差 [cm]。
   扫 Δt：斜率 = 相对位姿漂移速率，截距 = 单帧噪声。

L3 绝对点云偏差（vs 参考真值）
   unaligned：各系统点云按官方评测同款"初始化级 ENU 配准"（起点 GNSS 作原点 +
     前 30 s 位移 2D Kabsch 求 yaw，**不做全轨迹拟合**）落进同一 ENU 系，直接量到
     参考面的偏差 —— 就是"实际建出来的图偏多少"，含全局配准误差。
   aligned：再用参考面做稳健 6DoF point-to-plane 刚体拟合后量残差 —— 剔除全局
     刚体误差，只剩图内部畸变。两者之差 = 全局配准贡献，并同时输出 δt/δθ。
   注：参考面来自 FAST-LIVO2 自己的图，它自身的绝对误差**不**计入本层（只能由
   它的 RTK ATE 界定，见 HANDOVER）。

阶段（缓存在 --out，可单独重跑）：
  ref     FAST-LIVO2 稠密 PCD -> ENU -> 体素局部平面模型
  p1      扫一遍源 bag：逐系统记录点到参考面的局部坐标
  p2      跨帧配对（L2）
  report  汇总 JSON + PNG
"""
import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from frame_color_from_scans import read_pcd_xyz          # noqa: E402
from rtk_traj_prep import CDR                            # noqa: E402

DATA = Path('/home/ywj/slam_data/rtk_slam')
EVAL = DATA / 'eval_ref'
SRC_BAG = DATA / 'construction_seq2_r2'
SEQ = 'construction_seq2'
OUT = DATA / 'pcdev'

# ----  rig 标定：calib.yaml（lidar/IMU/camera 在同一车体上，四个系统共用）----
T_IL = np.array([0.011, 0.023, -0.044])      # lidar 原点在 IMU 系下（R=I）
T_IB = np.array([-0.073, -0.023, -0.172])    # base center 在 IMU 系下
T_BL = T_IL - T_IB                            # lidar 原点在 base(center) 系下
RCL = np.array([0.00108687, 0.99999887, -0.00103715,
                -0.38365052, -0.00054080, -0.92347820,
                -0.92347772, 0.00140160, 0.38364949]).reshape(3, 3)
PCL = np.array([-0.000974, -0.084395, -0.031208])       # lidar 原点在大图相机系下
FX, FY, CX, CY = 890.605, 890.619, 780.972, 589.790
IW, IH = 1600, 1200

SYSTEMS = ['fast_lio_sam', 'super_lio', 'super_lio_beam', 'super_lio_beam3', 'fast_livo2']
REF_SYS = 'fast_livo2'
RANGE_EDGES = np.array([0.0, 3.0, 5.0, 8.0, 12.0, 18.0, 25.0, 40.0])
GAPS = [0.2, 0.5, 1.0, 2.0, 5.0]
NFR = 5878                       # /livox/lidar 帧数（实测）


# ---------------------------------------------------------------- 小工具
def bins(v, edges):
    return np.clip(np.searchsorted(edges, v) - 1, 0, len(edges) - 2)


def robust(x):
    """(σ from MAD, 中位|x|, n)。"""
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if len(x) < 5:
        return float('nan'), float('nan'), len(x)
    return (float(1.4826 * np.median(np.abs(x - np.median(x)))),
            float(np.median(np.abs(x))), int(len(x)))


def basis(n):
    u = np.cross(n, np.array([0.0, 0.0, 1.0]))
    u[np.linalg.norm(u, axis=1) < 1e-6] = np.array([1.0, 0.0, 0.0])
    u /= np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-9)
    v = np.cross(n, u)
    return u, v


def vkey(ix, iy, iz):
    return (ix + 2048) + (iy + 2048) * 4096 + (iz + 2048) * (4096 * 4096)


def tum_poses(method, seq=SEQ):
    f = EVAL / 'trajectories' / method / seq / 'traj_offline.txt'
    if not f.exists():
        return None
    a = np.loadtxt(f)
    return a


# ---------------------------------------------------------------- 位姿查表
class Pose:
    """按**逐点时间**取位姿：一帧 0.1 s 恰好横跨 ~1 个里程计采样周期，
    不补偿车体在扫描期间的运动，会把运动直接写进"点云散布"。
    实现：按 bracket 分组，组内做两个刚体变换的线性混合（0.1 s 内一阶近似，
    误差远小于本层要分辨的 cm 级）。"""

    def __init__(self, tum):
        self.ts = tum[:, 0]
        self.pos = tum[:, 1:4].astype(np.float64)
        self.R = Rotation.from_quat(tum[:, 4:8]).as_matrix()

    def at(self, t0, offs, p):
        tq = t0 + offs
        k = np.clip(np.searchsorted(self.ts, tq) - 1, 0, len(self.ts) - 2)
        out = np.empty((len(p), 3))
        for kk in np.unique(k):
            m = k == kk
            span = max(self.ts[kk + 1] - self.ts[kk], 1e-9)
            w = np.clip((tq[m] - self.ts[kk]) / span, 0.0, 1.0)[:, None]
            pm = p[m]
            a = pm @ self.R[kk].T + self.pos[kk]
            b = pm @ self.R[kk + 1].T + self.pos[kk + 1]
            out[m] = a + w * (b - a)
        return out

    def rigid(self, t):
        """t 时刻最近邻位姿（帧首），返回 (R, s)。"""
        k = int(np.clip(np.searchsorted(self.ts, t) - 1, 0, len(self.ts) - 1))
        return self.R[k], self.pos[k]

    def cover(self, t0, offs):
        if len(offs) == 0:
            return False
        tq = t0 + offs
        return tq[0] >= self.ts[0] and tq[-1] <= self.ts[-1]


# ---------------------------------------------------------------- 雷达帧
class Scans:
    """源 bag 的 /livox/lidar（CustomMsg）流式读取。offset_time 单调、跨度
    100.2 ms，点时间 = 包头 ts + offset。20256 点/帧里约 58% 是零坐标占位。"""

    def __init__(self, bag=SRC_BAG, rng_min=0.5, rng_max=45.0):
        import sqlite3
        self.con = sqlite3.connect(str(sorted(Path(bag).glob('*.db3'))[0]))
        tid = self.con.execute("select id from topics where name='/livox/lidar'").fetchone()[0]
        self.cur = self.con.execute('select data from messages where topic_id=? order by timestamp',
                                    (tid,))
        self.rng = (rng_min, rng_max)

    def close(self):
        self.cur.close()
        self.con.close()

    def __iter__(self):
        for (data,) in self.cur:
            c = CDR(data)
            t, _ = c.header()
            c.u64()                      # timebase
            c.u32()                      # point_num
            c.u8()                       # lidar_id
            c.o += 3                     # rsvd[3]（与 rtk_verify_custommsg.py 同布局）
            n = c.u32()                  # seq_len
            # 每点 20 字节，但整包尾部少 1 个 pad 字节（实测），不足则补零，
            # 避免 as_strided 越界读到 blob 之外的内存。
            need = n * 20
            a = np.frombuffer(data, np.uint8, offset=c.o)
            if len(a) < need:
                a = np.concatenate([a, np.zeros(need - len(a), np.uint8)])
            else:
                a = a[:need]
            a = np.lib.stride_tricks.as_strided(
                a, shape=(n, 20), strides=(a.itemsize, a.itemsize))
            off = np.ascontiguousarray(a[:, 0:4]).view('<u4').ravel() * 1e-9
            mag = np.ascontiguousarray(a[:, 4:16]).view('<f4').reshape(n, 3)
            # 占位/异常点会出现 1e30 级坐标，直接 norm 会 float32 溢出报警；
            # 先裁剪到 ±100 m（本评测最大 45 m），溢出即消除，超界的照常滤掉。
            r = np.linalg.norm(np.clip(mag, -100.0, 100.0), axis=1)
            p = mag.astype(np.float64)
            keep = np.isfinite(r) & (r >= self.rng[0]) & (r <= self.rng[1])
            yield t, p[keep], off[keep]


def scan_iter(**kw):
    """流式遍历雷达帧，用完立刻关闭 sqlite（否则 10 GB bag 的 fd + 页缓存
    会一直挂在进程上）。"""
    sc = Scans(**kw)
    try:
        for x in sc:
            yield x
    finally:
        sc.close()


# =================================================================== ref
def stage_ref(a):
    fl2 = EVAL / 'trajectories' / REF_SYS / SEQ
    f = fl2 / 'world_to_enu.json'
    if not f.exists():
        raise SystemExit('缺 %s：先用 rtk_traj_prep.py 导出 %s 的 ENU 轨迹' % (f, REF_SYS))
    P = read_pcd_xyz(a.ref_pcd).astype(np.float64)
    P = P[np.isfinite(P).all(1)]
    w2e = json.loads(f.read_text())
    R, t = np.array(w2e['R']), np.array(w2e['t'])
    P = P @ R.T + t
    print('参考云 %s：%d 点 -> ENU 跨度 E %.1f~%.1f N %.1f~%.1f U %.1f~%.1f'
          % (os.path.basename(a.ref_pcd), len(P), P[:, 0].min(), P[:, 0].max(),
             P[:, 1].min(), P[:, 1].max(), P[:, 2].min(), P[:, 2].max()))

    e = a.edge
    ix = np.floor(P / e).astype(np.int64)
    key = vkey(ix[:, 0], ix[:, 1], ix[:, 2])
    uk, inv = np.unique(key, return_inverse=True)
    m = len(uk)
    cnt = np.bincount(inv, minlength=m).astype(np.float64)
    cen = np.stack([np.bincount(inv, weights=P[:, c], minlength=m) for c in range(3)], 1) / \
        np.maximum(cnt[:, None], 1)
    cov = np.zeros((m, 3, 3))
    for i in range(3):
        for j in range(i, 3):
            v = np.bincount(inv, weights=P[:, i] * P[:, j], minlength=m) / np.maximum(cnt, 1) \
                - cen[:, i] * cen[:, j]
            cov[:, i, j] = cov[:, j, i] = v
    ev, evec = np.linalg.eigh(cov)
    thick = np.sqrt(np.maximum(ev[:, 0], 0.0))
    # 判据自适应：参考云是 5 cm 体素化输出，0.25 m 体素里往往只有十几到几十个点，
    # 固定 min_pts=60 会把几乎所有体素挡掉（实测只剩 ~70 个面，几乎全部命中落在
    # 0–5 m）。这里按 --min-planes-ref 逐级放宽，直到拿到足够多的面。
    presets = [(a.min_pts, a.planar, a.max_thick),
               (max(a.min_pts // 2, 5), a.planar * 1.5, a.max_thick * 1.5),
               (max(a.min_pts // 4, 3), a.planar * 2.5, a.max_thick * 2.0)]
    for gi, (mp, pl, th) in enumerate(presets):
        ok = ((cnt >= mp) & (ev[:, 0] <= pl * np.maximum(ev[:, 2], 1e-12))
              & (thick <= th))
        if ok.sum() >= a.min_planes_ref or gi == len(presets) - 1:
            break
    sel = np.where(ok)[0]
    nrm = evec[sel][:, :, 0].copy()
    nrm[nrm[:, 2] < 0] *= -1.0                       # 法向符号统一（不影响偏差）
    nn = nrm / np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-9)
    U, V = basis(nn)
    np.savez_compressed(
        OUT / 'ref_planes.npz',
        key=uk[sel].astype(np.int64), cen=cen[sel].astype(np.float64),
        normal=nn.astype(np.float64), u=U.astype(np.float64), v=V.astype(np.float64),
        thick=thick[sel].astype(np.float32), npts=cnt[sel].astype(np.int32),
        edge=e, n_vox=np.int64(m), n_all=np.int64(len(P)),
        n_pts_ref=np.int64(cnt[sel].sum()))
    print('参考面 %d / 体素 %d（档位 %d/%d: min_pts=%d, planar<=%.3g, thick<=%.3f m）'
          ' 覆盖参考点 %.1f%%' % (len(sel), m, gi + 1, len(presets), mp, pl, th,
                                  100 * cnt[sel].sum() / len(P)))
    if len(sel) < 500:
        print('  [警告] 参考面只有 %d 个，距离/切向的层间统计会很不稳' % len(sel))


def load_ref():
    d = np.load(OUT / 'ref_planes.npz')
    sk = np.sort(d['key'])
    order = np.argsort(d['key'])
    return d, sk, order


# =================================================================== p1
def stage_p1(a):
    d, sk, order = load_ref()
    e = float(d['edge'])
    cen, nrm, U, V = d['cen'], d['normal'], d['u'], d['v']
    plist = order                                  # 排序键 -> 平面 id
    sysm = {}
    for nm in a.systems:
        t = tum_poses(nm)
        if t is None:
            print('  跳过 %s（无 traj_offline.txt）' % nm)
            continue
        sysm[nm] = Pose(t)
    print('参与系统：%s' % sorted(sysm))
    acc = {nm: dict(vox=[], dn=[], du=[], dv=[], rng=[], fr=[]) for nm in sysm}
    nuse = {nm: 0 for nm in sysm}
    ntot = {nm: 0 for nm in sysm}
    kf, kp = a.frame_stride, a.point_stride
    for fi, (t0, p, off) in enumerate(scan_iter()):
        if fi % kf:
            continue
        p, off = p[::kp], off[::kp]
        rr = np.linalg.norm(p, axis=1)
        for nm, pl in sorted(sysm.items()):
            if not pl.cover(t0, off):
                continue
            ntot[nm] += len(p)
            pw = pl.at(t0, off, p)
            ix = np.floor(pw / e).astype(np.int64)
            k = vkey(ix[:, 0], ix[:, 1], ix[:, 2])
            pos = np.searchsorted(sk, k)
            hit = (pos < len(sk)) & (sk[np.clip(pos, 0, len(sk) - 1)] == k)
            pid = plist[np.clip(pos, 0, len(sk) - 1)]
            rel = pw - cen[pid]
            dn = np.einsum('ni,ni->n', rel, nrm[pid])
            keep = hit & (np.abs(dn) <= a.dtol)
            if not keep.any():
                continue
            rel = rel[keep]
            q = acc[nm]
            q['vox'].append(pid[keep].astype(np.int32))
            q['dn'].append(dn[keep].astype(np.float32))
            q['du'].append(np.einsum('ni,ni->n', rel, U[pid[keep]]).astype(np.float32))
            q['dv'].append(np.einsum('ni,ni->n', rel, V[pid[keep]]).astype(np.float32))
            q['rng'].append(rr[keep].astype(np.float32))
            q['fr'].append(np.full(int(keep.sum()), fi // kf, np.int32))
            nuse[nm] += int(keep.sum())
        if (fi // kf) % 100 == 0:
            print('  帧 %d/%d  累计记录 %d' % (fi // kf, NFR // kf, sum(nuse.values())))
    for nm in sysm:
        z = {k: np.concatenate(v) for k, v in acc[nm].items()}
        np.savez_compressed(OUT / ('p1_%s.npz' % nm), **z)
        print('%-16s 投影点 %d，落参考面 %d（%.1f%%），命中参考面 %d/%d 个'
              % (nm, ntot[nm], len(z['dn']), 100 * len(z['dn']) / max(ntot[nm], 1),
                 len(np.unique(z['vox'])), len(cen)))


# =================================================================== p2
def project_lidar(p_l):
    """lidar 系点 -> 大图相机像素 + 深度。"""
    c = p_l @ RCL.T + PCL
    z = c[:, 2]
    u = np.full((len(c), 2), np.nan)
    g = z > 0.5
    u[g, 0] = FX * c[g, 0] / z[g] + CX
    u[g, 1] = FY * c[g, 1] / z[g] + CY
    inside = g & (u[:, 0] >= 0) & (u[:, 0] < IW) & (u[:, 1] >= 0) & (u[:, 1] < IH)
    return u, z, inside


def stage_p2(a):
    d, sk, order = load_ref()
    cen, nrm, e = d['cen'], d['normal'], float(d['edge'])
    tree = cKDTree(cen)
    sysm = {}
    for nm in a.systems:
        t = tum_poses(nm)
        if t is not None:
            sysm[nm] = Pose(t)
    names = sorted(sysm)
    kf = a.frame_stride
    nfr = NFR // kf
    rng = np.random.default_rng(a.seed)
    pairs, need = [], set()
    for g in GAPS:
        kg = max(1, int(round(g / (0.1 * kf))))
        if kg > nfr - 25:
            print('  Δt=%.1f s 在 frame_stride=%d（采样 %d 帧）下无法配对，跳过' % (g, kf, nfr))
            continue
        for _ in range(a.pairs_per_gap):
            i = int(rng.integers(10, nfr - kg - 10))
            pairs.append((i, i + kg, g))
            need.add(i)
            need.add(i + kg)
    need = sorted(need)
    print('配对 %d，需帧 %d（点步长 %d）' % (len(pairs), len(need), a.point_stride))
    want = {fi * kf: fi for fi in need}
    fr = {}
    for fi, (t0, p, off) in enumerate(scan_iter()):
        if fi in want:
            fr[want[fi]] = (t0, p[::a.point_stride], off[::a.point_stride])
        if len(fr) == len(need):
            break
    res = {nm: dict(gap=[], rng=[], dpl=[], dz=[], dpx=[], cos=[]) for nm in names}
    for (i, j, g) in pairs:
        t0i, pi, oi = fr[i]
        t0j, pj, oj = fr[j]
        for nm in names:
            pl = sysm[nm]
            wi = pl.at(t0i, oi, pi)
            wj = pl.at(t0j, oj, pj)
            # --- point-to-plane（法向，抗切向吸附）
            dj, jj = tree.query(wj, k=1)
            okj = dj <= 1.5 * e
            tr3 = cKDTree(wj)
            dd, nn = tr3.query(wi, k=a.knn + 1)
            dd = dd[:, 1:]
            nn = nn[:, 1:]
            best = np.argmin(dd, 1)
            ar = np.arange(len(wi))
            nbr = nn[ar, best]
            dist = dd[ar, best]
            ok = okj[nbr] & (dist < a.max_nn)
            rel = wi[ok] - wj[nbr[ok]]
            n2 = nrm[jj[nbr[ok]]]
            dpl = np.abs(np.einsum('nk,nk->n', rel, n2))
            cs = np.abs(np.einsum('nk,nk->n', rel, n2)) / np.maximum(
                np.linalg.norm(rel, axis=1), 1e-9)
            # --- 虚拟相机重投影：帧 i 的点与帧 j 自己的点都投进帧 j 时刻的相机
            Rj, sj = pl.rigid(t0j)
            q_l = (wi - sj) @ Rj - T_BL
            q_u, q_z, q_in = project_lidar(q_l)
            b_l = (wj - sj) @ Rj - T_BL
            b_u, b_z, b_in = project_lidar(b_l)
            if int(b_in.sum()) < 200:
                continue
            t2 = cKDTree(b_u[b_in])
            idx_ok = np.where(ok)[0]
            kk = np.where(q_in[idx_ok])[0]           # 帧 i 预测点中真正落在画面内的
            if len(kk) < 20:
                continue
            dq, bi2 = t2.query(q_u[idx_ok[kk]], k=1)
            keep2 = dq < a.max_px
            sel = idx_ok[kk[keep2]]
            k_ok = kk[keep2]                         # dpl/cs 是以 ok 长度为下标
            nb2 = bi2[keep2]
            bz = b_z[b_in][nb2]
            bu = b_u[b_in][nb2]
            R_ = res[nm]
            n_ = len(sel)
            if n_ < 20:
                continue
            R_['gap'].append(np.full(n_, g))
            R_['rng'].append(np.linalg.norm(q_l[sel], axis=1).astype(np.float32))
            R_['dpl'].append(dpl[k_ok].astype(np.float32))
            R_['dz'].append(np.abs(q_z[sel] - bz).astype(np.float32))
            R_['dpx'].append(np.linalg.norm(q_u[sel] - bu, axis=1).astype(np.float32))
            R_['cos'].append(cs[k_ok].astype(np.float32))
    for nm in names:
        if not res[nm]['gap']:
            print('%-16s L2 无样本' % nm)
            continue
        z = {k: np.concatenate(v) for k, v in res[nm].items()}
        np.savez_compressed(OUT / ('p2_%s.npz' % nm), **z)
        print('%-16s L2 样本 %d' % (nm, len(z['dpl'])))


# =================================================================== report
def fit_range(x, y, n):
    """σ²(R) = σ0² + (σθ·R)²，按每 bin 样本数加权最小二乘（拟合 σ² 本身，线性）。"""
    ok = np.isfinite(y) & (n > 300)
    if ok.sum() < 3:
        return {}
    A = np.stack([np.ones(int(ok.sum())), x[ok] ** 2], 1)
    w = np.sqrt(n[ok].astype(float))
    coef, *_ = np.linalg.lstsq(A * w[:, None], (y[ok] ** 2) * w, rcond=None)
    s0 = float(np.sqrt(max(coef[0], 0)))
    th = float(np.degrees(np.sqrt(max(coef[1], 0) * 1e-4)))
    pred = np.sqrt(np.maximum(coef[0] + coef[1] * x ** 2, 0))
    fit = A @ coef
    ss_r = float(np.sum(w ** 2 * (y[ok] ** 2 - fit) ** 2))
    ss_t = float(np.sum(w ** 2 * (y[ok] ** 2 - np.average(y[ok] ** 2, weights=w ** 2)) ** 2))
    return dict(sigma0_cm=round(s0, 3), theta_deg=round(th, 4),
                sigma_cm_at={'R=%.0f' % v: round(float(np.sqrt(max(coef[0] + coef[1] * v * v,
                                                                    0))), 3)
                             for v in (5, 10, 20, 30)},
                r2=round(1 - ss_r / max(ss_t, 1e-12), 3), bins=int(ok.sum()))


def align6d(cen, nrm, U, V, sd, su, sv, cnt, use, trim=3.0, rounds=3):
    """稳健 6DoF point-to-plane 刚体拟合 + 残差剔除。对应关系 = 每个参考面上该系统
    的点质心（由局部坐标 (dn,du,dv) 的均值还原）。

    改造点：单轮 lstsq 会被极少数离群面拖飞（实测 δθ 到 −150°、对齐后 MAD 反而
    涨到几十至两千厘米）。这里改用 MAD 截断迭代：每轮拟一次、剔除残差超
    median+trim*1.4826*MAD 的面，再在 inlier 上重拟。返回 inlier 数供调用方判断
    这次对齐是否可信。"""
    k = np.where(use)[0]
    nk = np.maximum(cnt[k], 1.0)
    pc = cen[k] + (sd[k] / nk)[:, None] * nrm[k] + (su[k] / nk)[:, None] * U[k] + \
        (sv[k] / nk)[:, None] * V[k]

    def _fit(kk):
        R = np.eye(3)
        t = np.zeros(3)
        w = np.sqrt(np.maximum(cnt[k[kk]], 1.0))
        for _ in range(10):
            # 旋转雅可比必须用**已变换后的点** v 与法向叉乘。若改用 q=v−cen，会
            # 多出 cross(cen,n)：参考面 cen 在 ENU 下可达数十至数百米，这一项把
            # 迭代推到 δt≈48 m / δθ≈100° 的发散解（实测）。
            v = pc[kk] @ R.T + t
            q = v - cen[k[kk]]
            A = np.concatenate([nrm[k[kk]], np.cross(v, nrm[k[kk]])], 1)
            y = -np.einsum('ni,ni->n', q, nrm[k[kk]])
            coef, *_ = np.linalg.lstsq(A * w[:, None], y * w, rcond=None)
            dR = Rotation.from_rotvec(coef[3:]).as_matrix()
            R = dR @ R
            t = dR @ t + coef[:3]
            if np.linalg.norm(coef[3:]) < 1e-6 and np.linalg.norm(coef[:3]) < 1e-5:
                break
        return R, t

    idx = np.arange(len(k))
    R, t = _fit(idx)
    for _ in range(rounds):
        # 残差只能在当前 inlier 子集 idx 上算：上一轮已把 idx 收窄，若仍用全量
        # len(k) 计算，后面的 idx[keep] 会因维度不一致直接抛 IndexError。
        q = (pc[idx] @ R.T + t) - cen[k[idx]]
        r = np.abs(np.einsum('ni,ni->n', q, nrm[k[idx]]))
        med = np.median(r)
        mad = np.median(np.abs(r - med)) * 1.4826
        keep = r <= max(med + trim * max(mad, 1e-6), 1e-3)
        if keep.all() or keep.sum() < 10:
            break
        idx = idx[keep]
        R, t = _fit(idx)

    q = (pc[idx] @ R.T + t) - cen[k[idx]]
    resid = np.abs(np.einsum('ni,ni->n', q, nrm[k[idx]]))
    return (R, t), resid, (t, np.degrees(Rotation.from_matrix(R).as_rotvec())), int(len(idx))


def stage_report(a):
    d, sk, order = load_ref()
    cen, nrm, U, V = d['cen'], d['normal'], d['u'], d['v']
    out = dict(ref=dict(n_planes=int(len(cen)), edge=float(d['edge']),
                        ref_points=int(d['n_pts_ref']), n_vox=int(d['n_vox']),
                        coverage=round(float(d['n_pts_ref']) / max(int(d['n_all']), 1), 3),
                        source='FAST-LIVO2 稠密建图（自身绝对误差由 RTK ATE 界定）'))
    names = [nm for nm in a.systems if (OUT / ('p1_%s.npz' % nm)).exists()]
    for nm in names:
        z = np.load(OUT / ('p1_%s.npz' % nm))
        vox, dn = z['vox'], z['dn'].astype(np.float64)
        du, dv = z['du'].astype(np.float64), z['dv'].astype(np.float64)
        rr, frr = z['rng'].astype(np.float64), z['fr']
        M = len(cen)
        cnt = np.bincount(vox, minlength=M).astype(np.float64)
        sd = np.bincount(vox, weights=dn, minlength=M)
        su = np.bincount(vox, weights=du, minlength=M)
        sv = np.bincount(vox, weights=dv, minlength=M)
        use = cnt >= a.min_per_plane
        mu = np.divide(sd, cnt, out=np.zeros(M), where=cnt > 0)
        # ---- L1
        r = dn - mu[vox]
        # ---- 层间距 layer_std：同一面内"逐帧均值"的点数加权散布 = 纯重影/分层。
        #      先把点按 (面, 帧) 分组求均值（组内点数 >= min_per_frame 才可信），
        #      再在面内对这些帧均值求加权 std，最后按面的平均距离分箱取中位。
        fkey = vox.astype(np.int64) * 1000003 + frr.astype(np.int64)
        _, ig = np.unique(fkey, return_inverse=True)
        ng = int(ig.max()) + 1
        cf = np.bincount(ig, minlength=ng).astype(np.float64)
        mf = np.bincount(ig, weights=dn, minlength=ng) / np.maximum(cf, 1.0)
        rf = np.bincount(ig, weights=rr, minlength=ng) / np.maximum(cf, 1.0)
        pf = np.bincount(ig, weights=vox.astype(np.float64), minlength=ng) / np.maximum(cf, 1.0)
        g = cf >= a.min_per_frame                      # 组（面×帧）可信
        P, W, F, D = pf[g].astype(np.int64), cf[g], mf[g], rf[g]
        w1 = np.bincount(P, weights=W, minlength=M)
        lstd = np.full(M, np.nan)
        ldist = np.full(M, np.nan)
        nfg = np.bincount(P, minlength=M)
        nz = w1 > 0
        m1 = np.bincount(P, weights=W * F, minlength=M)
        m2 = np.bincount(P, weights=W * F * F, minlength=M)
        ds = np.bincount(P, weights=W * D, minlength=M)
        mean_f = np.divide(m1, w1, out=np.zeros(M), where=nz)
        lvar = np.maximum(np.divide(m2, w1, out=np.zeros(M), where=nz) - mean_f ** 2, 0.0)
        lstd[nz] = np.sqrt(lvar[nz])
        ldist[nz] = np.divide(ds, w1, out=np.zeros(M), where=nz)[nz]
        okL = nz & (nfg >= 3)
        L1 = {}
        for b in range(len(RANGE_EDGES) - 1):
            lo, hi = RANGE_EDGES[b], RANGE_EDGES[b + 1]
            m = use[vox] & (rr >= lo) & (rr < hi)
            s, mad, n = robust(r[m])
            ml = okL & np.isfinite(lstd) & (ldist >= lo) & (ldist < hi)
            L1['%.0f-%.0f' % (lo, hi)] = dict(
                n=n, sigma_cm=round(s * 100, 3) if n else None,
                p50_cm=round(mad * 100, 3) if n else None,
                p90_cm=round(float(np.percentile(np.abs(r[m]), 90)) * 100, 3) if n > 20 else None,
                n_layers=int(ml.sum()),
                layer_cm=round(float(np.median(lstd[ml])) * 100, 3) if ml.sum() >= 10 else None)
        lay = lstd[okL & np.isfinite(lstd)]
        layers = dict(n_planes=int(okL.sum()),
                      med_cm=round(float(np.median(lay)) * 100, 3) if len(lay) else None,
                      p90_cm=round(float(np.percentile(lay, 90)) * 100, 3) if len(lay) > 10 else None)
        xs = (RANGE_EDGES[:-1] + RANGE_EDGES[1:]) / 2
        ys = np.array([L1['%.0f-%.0f' % (RANGE_EDGES[b], RANGE_EDGES[b + 1])]['sigma_cm'] or np.nan
                       for b in range(len(xs))])
        ns = np.array([L1['%.0f-%.0f' % (RANGE_EDGES[b], RANGE_EDGES[b + 1])]['n']
                       for b in range(len(xs))])
        # ---- L3
        R3 = dict(planes_used=int(use.sum()),
                  bias_cm=dict(med=round(float(np.median(np.abs(mu[use]))) * 100, 3),
                               p90=round(float(np.percentile(np.abs(mu[use]), 90)) * 100, 3)),
                  centroid_dev_cm=dict(med=round(float(np.median(np.hypot(
                      np.hypot(su[use] / cnt[use], sv[use] / cnt[use]), mu[use]))) * 100, 3),
                      p90=round(float(np.percentile(np.hypot(np.hypot(
                          su[use] / cnt[use], sv[use] / cnt[use]), mu[use]), 90)) * 100, 3)),
                  normal_cm=dict(mad=round(robust(dn[use[vox]])[0] * 100, 3),
                                 p50=round(robust(dn[use[vox]])[1] * 100, 3),
                                 p90=round(float(np.percentile(np.abs(dn[use[vox]]), 90)) * 100,
                                           3)),
                  tang_cm=dict(p50=round(float(np.percentile(
                      np.hypot(du, dv)[use[vox]], 50)) * 100, 3),
                      p90=round(float(np.percentile(
                          np.hypot(du, dv)[use[vox]], 90)) * 100, 3)))
        # 对齐门控：面太少时 6DoF 拟合欠定，硬拟会得到 δθ=−150° 这类毫无意义的解，
        # 并把"对齐后 MAD"抬到几十甚至两千厘米。面数不足或纠偏量超限一律拒绝对齐。
        n_planes = int(use.sum())
        aligned_ok = n_planes >= a.min_align_planes
        refuse = '' if aligned_ok else '面数不足(%d<%d)' % (n_planes, a.min_align_planes)
        if aligned_ok:
            (R, t), resid, dof, n_in = align6d(cen, nrm, U, V, sd, su, sv, cnt, use, a.trim)
            dt_m = np.linalg.norm(dof[0]) * 100
            dth_deg = float(np.degrees(np.linalg.norm(Rotation.from_matrix(R).as_rotvec())))
            if dt_m > a.max_align_t_cm or dth_deg > a.max_align_deg:
                aligned_ok = False
                refuse = '纠偏超限(δt=%.1fcm>%.0f 或 δθ=%.2f°>%.1f)' % (
                    dt_m, a.max_align_t_cm, dth_deg, a.max_align_deg)
        if not aligned_ok:
            R, t = np.eye(3), np.zeros(3)
            dof = (np.zeros(3), np.zeros(3))
            n_in = 0
        # 点在"所在参考面局部系"里的偏移矢量（法向 dn + 切向 du/dv）还原到世界，
        # 才能把全局刚体拟合 R,t 作用上去。dn/du/dv 是逐点 1-D，必须升一维再乘 (n,3)。
        rel = dn[:, None] * nrm[vox] + du[:, None] * U[vox] + dv[:, None] * V[vox]
        p_al = (cen[vox] + rel) @ R.T + t
        u_m = use[vox]          # 与 L3_unaligned 同口径：只用可用参考面上的落点
        dn_al = np.einsum('ni,ni->n', p_al - cen[vox], nrm[vox])[u_m]
        d_tan_al = np.linalg.norm(p_al - cen[vox], axis=1)[u_m]
        L3a = dict(normal_mad_cm=round(robust(dn_al)[0] * 100, 3),
                   normal_p50_cm=round(robust(dn_al)[1] * 100, 3),
                   normal_p90_cm=round(float(np.percentile(np.abs(dn_al), 90)) * 100, 3),
                   dist_p50_cm=round(float(np.percentile(d_tan_al, 50)) * 100, 3),
                   planes=n_planes, aligned=bool(aligned_ok),
                   align_inliers=int(n_in), refused_reason=refuse)
        # ---- L2
        L2 = {}
        pf = OUT / ('p2_%s.npz' % nm)
        if pf.exists():
            q = np.load(pf)
            for g in GAPS:
                m = q['gap'] == g
                if m.sum() < 50:
                    continue
                L2['%.1f' % g] = dict(
                    n=int(m.sum()),
                    dplane_cm=dict(mad=round(robust(q['dpl'][m])[0] * 100, 3),
                                   p50=round(float(np.median(q['dpl'][m])) * 100, 3),
                                   p90=round(float(np.percentile(q['dpl'][m], 90)) * 100, 3)),
                    depth_cm=dict(mad=round(robust(q['dz'][m])[0] * 100, 3),
                                  p50=round(float(np.median(q['dz'][m])) * 100, 3)),
                    pixel=dict(mad=round(robust(q['dpx'][m])[0], 3),
                               p50=round(float(np.median(q['dpx'][m])), 2),
                               p90=round(float(np.percentile(q['dpx'][m], 90)), 2)))
        out[nm] = dict(L1=L1, L1_fit=fit_range(xs, ys, ns), L1_layers=layers, L2=L2,
                       L3_unaligned=R3, L3_aligned=L3a,
                       align=dict(dt_cm=[round(v * 100, 2) for v in dof[0]],
                                  dtheta_deg=[round(v, 3) for v in dof[1]]))
        print('%-16s L1 σ0=%s cm σθ=%s° 层间距=%s cm | L3 未对齐 法向MAD=%s 偏置=%s | 对齐后 MAD=%s cm '
              '(δt=%.1fcm δθ=%.2f°) | L2 Δt=1s dplane=%s px=%s'
              % (nm, out[nm]['L1_fit'].get('sigma0_cm'), out[nm]['L1_fit'].get('theta_deg'),
                 layers.get('med_cm'),
                 R3['normal_cm']['mad'], R3['bias_cm']['med'], L3a['normal_mad_cm'],
                 np.linalg.norm(dof[0]) * 100, np.linalg.norm(dof[1]),
                 L2.get('1.0', {}).get('dplane_cm', {}).get('mad'),
                 L2.get('1.0', {}).get('pixel', {}).get('p50')))

    # ---- 噪声地板：参考系统与自身比对（同一份点云、同一套位姿）。
    # 它量的是"参考面厚度上限 8 cm + 体素量化 + 采样步长"共同决定的量级，是这一层
    # 能分辨到的最细刻度，**不是下界**：各系统覆盖的参考面子集不同，MAD 落在
    # 0.8~1.1 倍以内即说明差异已被本层噪声淹没，不能据此排序。
    nf = out.get(REF_SYS)
    if nf:
        floor = float(nf['L3_unaligned']['normal_cm']['mad'])
        out['noise_floor'] = dict(
            system=REF_SYS, normal_mad_cm=floor,
            bias_cm=nf['L3_unaligned']['bias_cm']['med'],
            note='参考系统自比对的残留量级，由参考面厚度上限(8 cm)/体素量化/采样密度'
                 '主导，代表 L3 的分辨刻度而非下界；各系统 vs_floor = 该系统法向 MAD '
                 '/ 该刻度，0.8~1.1 视为不可区分')
        for nm in names:
            if nm == REF_SYS:
                continue
            v = float(out[nm]['L3_unaligned']['normal_cm']['mad'])
            out[nm]['vs_floor'] = round(v / max(floor, 1e-9), 3)
        print('噪声地板（%s 自比对）：法向 MAD=%.3f cm，偏置=%.3f cm'
              % (REF_SYS, floor, nf['L3_unaligned']['bias_cm']['med']))

    (OUT / 'report.json').write_text(json.dumps(out, indent=1, ensure_ascii=False))
    print('-> %s/report.json' % OUT)
    plot(out, names)


def plot(out, names):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as ex:
        print('matplotlib 跳过：%s' % ex)
        return
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.4))
    for nm in names:
        L1 = out[nm]['L1']
        x = [(float(k.split('-')[0]) + float(k.split('-')[1])) / 2 for k in L1]
        y = [L1[k]['sigma_cm'] or np.nan for k in L1]
        ax[0].plot(x, y, 'o-', label=nm, ms=4)
        yl = [L1[k].get('layer_cm') or np.nan for k in L1]
        if np.isfinite(yl).any():
            ax[0].plot(x, yl, 's--', label=nm + ' 层间距', ms=3, alpha=.75)
        L2 = out[nm]['L2']
        if L2:
            g = [float(k) for k in L2]
            ax[1].plot(g, [L2[k]['dplane_cm']['mad'] for k in L2], 'o-', label=nm, ms=4)
            ax[1].plot(g, [L2[k]['pixel']['p50'] for k in L2], '--', label=nm + ' px', ms=3)
    ax[0].set(xscale='log', xlabel='range R (m)', ylabel='σ 法向散布 (cm)',
              title='L1 距离分层平面散布')
    ax[1].set(xscale='log', xlabel='帧间隔 Δt (s)', ylabel='point-to-plane MAD (cm) / 像素 (px)',
              title='L2 跨帧一致性（实线 cm，虚线 px）')
    w = 0.38
    for i, nm in enumerate(names):
        v = [out[nm]['L3_unaligned']['normal_cm']['mad'], out[nm]['L3_aligned']['normal_mad_cm']]
        ax[2].bar(np.arange(2) + (i - len(names) / 2) * w, v, w, label=nm)
    ax[2].set_xticks([0, 1])
    ax[2].set_xticklabels(['ENU 未对齐', 'SE(3) 对齐后'])
    ax[2].set(ylabel='法向偏差 MAD (cm)', title='L3 绝对点云偏差 vs FAST-LIVO2 参考')
    for k in range(3):
        ax[k].grid(alpha=.3)
        ax[k].legend(fontsize=6)
    fig.tight_layout()
    fig.savefig(OUT / 'layers.png', dpi=115)
    print('-> %s/layers.png' % OUT)


# =================================================================== main
def main():
    global OUT
    ap = argparse.ArgumentParser()
    ap.add_argument('--stage', default='all')
    ap.add_argument('--out', default=str(OUT))
    ap.add_argument('--ref-pcd', default=str(DATA / 'run_fl2_seq2_full/Cloud_merged.pcd'))
    ap.add_argument('--systems', nargs='*', default=SYSTEMS)
    ap.add_argument('--edge', type=float, default=0.25)
    ap.add_argument('--min-pts', type=int, default=15)
    ap.add_argument('--planar', type=float, default=0.08)
    ap.add_argument('--max-thick', type=float, default=0.08)
    ap.add_argument('--min-planes-ref', type=int, default=2000)
    ap.add_argument('--min-align-planes', type=int, default=500)
    ap.add_argument('--trim', type=float, default=3.0)
    ap.add_argument('--max-align-t-cm', type=float, default=100.0)
    ap.add_argument('--max-align-deg', type=float, default=5.0)
    ap.add_argument('--dtol', type=float, default=0.20)
    ap.add_argument('--min-per-plane', type=int, default=8)
    ap.add_argument('--min-per-frame', type=int, default=3)
    ap.add_argument('--frame-stride', type=int, default=6)
    ap.add_argument('--point-stride', type=int, default=3)
    ap.add_argument('--pairs-per-gap', type=int, default=120)
    ap.add_argument('--knn', type=int, default=4)
    ap.add_argument('--max-nn', type=float, default=0.35)
    ap.add_argument('--max-px', type=float, default=25.0)
    ap.add_argument('--seed', type=int, default=5)
    ap.add_argument('--max-frames', type=int, default=0)
    a = ap.parse_args()
    OUT = Path(a.out)
    OUT.mkdir(parents=True, exist_ok=True)
    for st in (['ref', 'p1', 'p2', 'report'] if a.stage == 'all' else [a.stage]):
        {'ref': stage_ref, 'p1': stage_p1, 'p2': stage_p2, 'report': stage_report}[st](a)


if __name__ == '__main__':
    main()
