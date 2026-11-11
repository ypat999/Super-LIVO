#!/usr/bin/env python3
"""把 Super-LIO 的 /lio/odom 转成 RTK-SLAM 官方评测要求的 ENU 轨迹。

评测协议（eval_ref/README.md）要求：
  trajectories/<method>/<seq>/traj_offline.txt   TUM: ts x y z qx qy qz qw
  trajectories/<method>/<seq>/enu_origin.json    [lat, lon, alt]

且位置必须在 **base center** 上（IMU→base: calib.yaml 的 base_center
= [-0.073,-0.023,-0.172]，在 IMU 系下），高度用 enu_origin.alt = 椭球高 + 48.22
（与官方 fast_lio_sam 基线 enu_origin [48.863,9.327,342.76] 同口径：
  342.76 ≈ 294.5 + 48.22）。

Super-LIO 的世界系是"初始化时刻的 IMU 系"（重力对齐、yaw 任意）。
本脚本做两件**初始化级**（非全轨迹拟合）的事：
  1) 原点：取轨迹起点时刻的 GNSS 位置作 ENU 原点；
  2) 航向：只用**开头 HEAD_SEC 秒**的 GNSS 与 SLAM 位移做 2D Kabsch 求 yaw。
不做任何全轨迹 SE(3) 对齐 —— 那正是官方要剔除的 Gap。

用法：
  python3 tools/rtk_traj_prep.py \
      --odom-bag /home/ywj/slam_data/rtk_slam/run_lio_seq2 \
      --src-bag  /home/ywj/slam_data/rtk_slam/construction_seq2_r2 \
      --method super_lio --seq construction_seq2 \
      --out-raw /home/ywj/slam_data/rtk_slam/super_lio_seq2_imu.txt
"""
import argparse
import json
import math
import sqlite3
import struct
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# IMU -> base center（IMU 系），来自 calib.yaml reference_offsets.base_center
T_IMU_BASE = np.array([-0.073, -0.023, -0.172])
# 注：不要给 enu_origin.alt 额外加大地水准面常数。官方基线
# fast_lio_sam/construction_seq2_enu_origin.json 的 alt=342.76 就等于起步时刻的
# GNSS 椭球高（本脚本实测 342.763），大地水准面改正由 coords.py 内部统一做。
HEAD_SEC = 30.0

EVAL_ROOT = Path('/home/ywj/slam_data/rtk_slam/eval_ref')


class CDR:
    """ROS2 CDR (little-endian, 4 字节封装头) 最小解析器。"""

    def __init__(self, buf):
        if buf[0:2] != b'\x00\x01':
            raise ValueError('not CDR-LE: %r' % buf[0:4])
        self.b = buf
        self.o = 4

    def _a(self, n):
        r = (self.o - 4) % n
        if r:
            self.o += n - r

    def i32(self):
        self._a(4)
        v = struct.unpack_from('<i', self.b, self.o)[0]
        self.o += 4
        return v

    def u32(self):
        self._a(4)
        v = struct.unpack_from('<I', self.b, self.o)[0]
        self.o += 4
        return v

    def u16(self):
        self._a(2)
        v = struct.unpack_from('<H', self.b, self.o)[0]
        self.o += 2
        return v

    def i8(self):
        v = struct.unpack_from('<b', self.b, self.o)[0]
        self.o += 1
        return v

    def u8(self):
        v = self.b[self.o]
        self.o += 1
        return v

    def f64(self, n=1):
        self._a(8)
        v = struct.unpack_from('<%dd' % n, self.b, self.o)
        self.o += 8 * n
        return np.array(v) if n > 1 else v[0]

    def u64(self):
        self._a(8)
        v = struct.unpack_from('<Q', self.b, self.o)[0]
        self.o += 8
        return v

    def string(self):
        n = self.u32()
        s = self.b[self.o:self.o + n]
        self.o += n
        return s.rstrip(b'\x00').decode('utf-8', 'replace')

    def header(self):
        sec = self.i32()
        nsec = self.u32()
        fid = self.string()
        return sec + nsec * 1e-9, fid


def read_odom(bag_dir, topic='/lio/odom'):
    """按 odom header 时间戳返回位姿。

    ⚠️ 必须自己排序：bag 里 messages.timestamp 是**录制时刻**，与 odom header
    里的雷达时刻差一个随负载变化的发布延迟。直接按录制顺序导出会得到
    非单调的时间戳（实测 224~287 处倒退、最大 5 s），任何"步长/速度/抖动"
    分析都会被假瞬移污染（最近邻匹配类评测不受影响，因为它对全序列取 argmin）。
    """
    db = sorted(Path(bag_dir).glob('*.db3'))[0]
    con = sqlite3.connect(str(db))
    row = con.execute("select id from topics where name=?", (topic,)).fetchone()
    if row is None:
        names = [r[0] for r in con.execute("select name from topics")]
        raise SystemExit('话题 %s 不在 %s 中，可选：%s' % (topic, bag_dir, names))
    tid = row[0]
    rows = []
    for rec_ts, data in con.execute(
            "select timestamp, data from messages where topic_id=? order by timestamp", (tid,)):
        c = CDR(data)
        t, _ = c.header()
        c.string()                       # child_frame_id
        p = c.f64(3)                     # pose.pose.position
        q = c.f64(4)                     # pose.pose.orientation (x,y,z,w)
        rec = rec_ts * 1e-9 if rec_ts > 1e14 else float(rec_ts)   # 兼容 ns / s 存储
        rows.append((t, rec, p, q))
    con.close()
    rows.sort(key=lambda r: r[0])
    ts = np.array([r[0] for r in rows])
    pos = np.array([r[2] for r in rows])
    quat = np.array([r[3] for r in rows])
    inv = int((np.diff(ts) < 0).sum())
    print('按 odom header 时间戳排序：%d 帧，倒退 %d 处' % (len(ts), inv))
    if inv:
        print('!! 排序后仍有倒退：同一时刻有重复发布，需去重')
    return ts, pos, quat


def read_gnss(bag_dir, topic='/gnss/fix'):
    db = sorted(Path(bag_dir).glob('*.db3'))[0]
    con = sqlite3.connect(str(db))
    tid = con.execute("select id from topics where name=?", (topic,)).fetchone()[0]
    ts, llh, st = [], [], []
    for (data,) in con.execute("select data from messages where topic_id=? order by timestamp", (tid,)):
        c = CDR(data)
        t, _ = c.header()
        stat = c.i8()
        c.u16()                          # service
        lat, lon, alt = c.f64(3)
        ts.append(t)
        llh.append((lat, lon, alt))
        st.append(stat)
    con.close()
    return np.array(ts), np.array(llh), np.array(st)


def latlon_to_utm(lat, lon, height, cm=9.0):
    """与 eval_ref/eval/coords.py 完全一致的 UTM 正算（ETRS89/GRS80）。"""
    a, f = 6378137.0, 1 / 298.257222101
    e2 = 2 * f - f * f
    ep2 = e2 / (1 - e2)
    lr, nr = math.radians(lat), math.radians(lon)
    cmr = math.radians(cm)
    N = a / math.sqrt(1 - e2 * math.sin(lr) ** 2)
    T = math.tan(lr) ** 2
    C = ep2 * math.cos(lr) ** 2
    A = math.cos(lr) * (nr - cmr)
    M = a * ((1 - e2 / 4 - 3 * e2 ** 2 / 64 - 5 * e2 ** 3 / 256) * lr
             - (3 * e2 / 8 + 3 * e2 ** 2 / 32 + 45 * e2 ** 3 / 1024) * math.sin(2 * lr)
             + (15 * e2 ** 2 / 256 + 45 * e2 ** 3 / 1024) * math.sin(4 * lr)
             - (35 * e2 ** 3 / 3072) * math.sin(6 * lr))
    E = 500000 + 0.9996 * N * (A + (1 - T + C) * A ** 3 / 6
                              + (5 - 18 * T + T ** 2 + 72 * C - 58 * ep2) * A ** 5 / 120)
    Nn = 0.9996 * (M + N * math.tan(lr) * (A ** 2 / 2
                   + (5 - T + 9 * C + 4 * C ** 2) * A ** 4 / 24
                   + (61 - 58 * T + T ** 2 + 600 * C - 330 * ep2) * A ** 6 / 720))
    if lat < 0:
        Nn += 10000000
    return E, Nn, height


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--odom-bag', required=True)
    ap.add_argument('--odom-topic', default='/lio/odom',
                    help='官方 FAST-LIVO2 用 /aft_mapped_to_init')
    ap.add_argument('--src-bag', required=True)
    ap.add_argument('--method', default='super_lio')
    ap.add_argument('--seq', default='construction_seq2')
    ap.add_argument('--out-raw', default='')
    ap.add_argument('--head-sec', type=float, default=HEAD_SEC)
    ap.add_argument('--eval-root', default=str(EVAL_ROOT))
    args = ap.parse_args()

    ts, pos, quat = read_odom(args.odom_bag, args.odom_topic)
    print('odom poses: %d, %.1f s (%.3f .. %.3f)' % (len(ts), ts[-1] - ts[0], ts[0], ts[-1]))

    # IMU -> base center
    Rm = Rotation.from_quat(quat).as_matrix()
    pos_base = pos + np.einsum('nij,j->ni', Rm, T_IMU_BASE)

    gts, gllh, gst = read_gnss(args.src_bag)
    print('gnss fixes: %d, status 分布 %s' % (len(gts), dict(zip(*np.unique(gst, return_counts=True)))))

    # 每个 SLAM 位姿匹配最近 GNSS（0.1 s 内）
    idx = np.searchsorted(gts, ts).clip(1, len(gts) - 1)
    left = np.abs(ts - gts[idx - 1]) < np.abs(ts - gts[idx])
    idx = np.where(left, idx - 1, idx)
    dt = np.abs(gts[idx] - ts)
    ok = dt < 0.1
    print('gnss 匹配率: %d/%d (中位 |dt| = %.1f ms)'
          % (ok.sum(), len(ts), np.median(dt[ok]) * 1e3))
    if not ok.any():
        raise SystemExit('GNSS 匹配失败，检查 /gnss/fix 时间戳')

    i0 = int(np.argmax(ok))                       # 第一个有 GNSS 的位姿
    lat0, lon0, alt0 = gllh[idx[i0]]
    utm0 = np.array(latlon_to_utm(lat0, lon0, alt0)[:2])

    def to_enu(k):
        E, N, _ = latlon_to_utm(*gllh[idx[k]])
        return np.array([E, N]) - utm0

    # ---- 只用开头 head_sec 秒估初始航向（2D Kabsch，无尺度、无全轨迹参与） ----
    good = ok & (gst[idx] >= 0)          # 窗口内只信有定位解（status>=0）的 GNSS
    sel = np.where(good & (ts <= ts[i0] + args.head_sec))[0]
    if len(sel) < 20:
        print('窗口内有效 GNSS 太少（%d），退回全部匹配点' % len(sel))
        sel = np.where(ok & (ts <= ts[i0] + args.head_sec))[0]
    g = np.array([to_enu(k) for k in sel])
    s = pos_base[sel, :2]
    g -= g.mean(0)
    s -= s.mean(0)
    U, _, Vt = np.linalg.svd(s.T @ g)
    R2 = Vt.T @ U.T
    if np.linalg.det(R2) < 0:
        Vt[-1] *= -1
        R2 = Vt.T @ U.T
    yaw = math.atan2(R2[1, 0], R2[0, 0])
    resid = np.linalg.norm((s @ R2.T) - g, axis=1)
    span = float(np.linalg.norm(g[-1] - g[0]))
    print('初始航向: yaw = %.3f° (窗口 %.0f s / %d 点 / GNSS 位移 %.1f m / 残差中位 %.2f m)'
          % (math.degrees(yaw), args.head_sec, len(sel), span, np.median(resid)))

    # ---- ENU 输出 ----
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                   [math.sin(yaw), math.cos(yaw), 0.0],
                   [0.0, 0.0, 1.0]])
    enu = (pos_base - pos_base[i0]) @ Rz.T
    q_enu = (Rotation.from_euler('z', yaw) * Rotation.from_quat(quat)).as_quat()

    if args.out_raw:
        raw = np.column_stack([ts, pos, quat])
        np.savetxt(args.out_raw, raw, fmt=['%.9f'] + ['%.6f'] * 3 + ['%.9f'] * 4)
        print('已写 %s（SLAM 原始系 IMU 位置）' % args.out_raw)

    outdir = Path(args.eval_root) / 'trajectories' / args.method / args.seq
    outdir.mkdir(parents=True, exist_ok=True)
    tum = np.column_stack([ts, enu, q_enu])
    np.savetxt(outdir / 'traj_offline.txt', tum,
               fmt=['%.9f'] + ['%.6f'] * 3 + ['%.9f'] * 4)
    origin = [lat0, lon0, alt0]
    (outdir / 'enu_origin.json').write_text(json.dumps(origin, indent=2))
    # 地图点云 -> ENU 的刚体变换（与上面的轨迹同口径：p_enu = R @ p_world + t）。
    # 点云是"世界系里的位置"，不是 base center，因此平移要把 base 杆臂一并扣掉，
    # 才能让图和轨迹落在同一个 ENU 原点。
    (outdir / 'world_to_enu.json').write_text(json.dumps(
        dict(R=Rz.tolist(), t=list(-Rz @ pos_base[i0])), indent=1))
    print('已写 %s' % outdir)
    print('enu_origin = [%.9f, %.9f, %.3f]' % (origin[0], origin[1], origin[2]))
    print('轨迹跨度 ENU: E %.1f~%.1f  N %.1f~%.1f  U %.1f~%.1f'
          % (enu[:, 0].min(), enu[:, 0].max(), enu[:, 1].min(), enu[:, 1].max(),
             enu[:, 2].min(), enu[:, 2].max()))


if __name__ == '__main__':
    main()
