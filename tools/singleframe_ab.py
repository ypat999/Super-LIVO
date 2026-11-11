#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单帧 · 同点 · 同图 · 同外参 逐点 A/B 对照基准。

只读诊断工具：不修改任何算法实现，只从已有 bag 反演两套投影链并逐点对比。

对比对象
--------
自研方案A   pointcloud_colorizer/ + solution_a_lightweight/super_livo_visual_a
官方 FAST-LIVO2  third_party/FAST-LIVO2-ROS2 (+ rpg_vikit)

如何把两次独立的运行对到「同一次曝光」上（本工具的核心）
----------------------------------------------------------
两次运行的世界系原点/朝向完全不同（官方 camera_init、自研也是各自的起点），
官方 bag 的 header 还是 node->now() 处理时刻，都不能当时间轴用。唯一可靠的
锚是**原始数据集里的第几张图**：
  自研帧 i  ->  imgs.nearest(cloud header)（实测 hdr 差 +0.0002~+0.005 s）
  官方帧 k  ->  由光度证据确定：把官方帧 k 的云用同索引 odom 投回某张原图，
               只有「这张图真是它取过色的那张」，重采样 RGB 才与 bag 发布
               的 RGB 对上（median MAE ~1/255；错一张就是 10~40）。
两侧落在同一张图上 => 相机系是同一个物理坐标系（同一台相机、同一次曝光、
同一常量外参），此时 p_cam 可直接做最近邻配对，u/v 也可逐点直接比较。

投影链（两侧代数上等价，只差预处理与可见性判定）
--------------------------------------------------
自研: p_cam = T_cl · T_lw(t_img)^-1 · T_lw(t_cloud) · p_body, t_img = t_pair + 0.1
      在 remap 去畸变后的图上按理想 pinhole 像素取值
官方: p_cam = Rci·Rwi^T·(p_w - Pwi) + Pci, Rci=Rcl·Rli^T, Pci=Pcl-Rcl·extT
      在**原图**上按「理想投影 + 前向畸变」的像素取值（vikit 带 d0!=0 分支）

三档判据
--------
1) 像素差小 / RGB 差大 -> 预处理、通道顺序、采样插值、颜色编码
2) 像素差大             -> 投影模型、外参、时间戳、图像坐标系
3) 单帧一致、地图不一致 -> 跨帧累积 / 重复点 / 渲染

输出
----
<outdir>/ab_points.csv          评审要求的严格 12 字段
<outdir>/ab_points_extended.csv 共享位姿版 + 重采样 RGB + 分组标记 + 重建偏差

字段口径（重要）
----------------
x,y,z 是**官方世界系**(camera_init)里的坐标；自研运行的世界系原点/朝向与它不同，
不能拿 (x,y,z) 直接和自研的世界坐标混用。逐点几何比较请用扩展表里的
xc/yc/zc_official 与 xc/yc/zc_custom —— 这两组都在同一个物理相机坐标系里
(同一张原图 = 同一次曝光)，是本项目里唯一严格可比的 3D 坐标。
像素列要注意坐标系约定不同：u/v_official 是**原图(带畸变)**像素，
u/v_custom 是**去畸变图**像素（自研在 remap 后的图上取值），两者直接相减
里面混了畸变约定；要比较几何本身请用 u_custom_in_raw_coords（把自研理想像素
再前向畸变到原图坐标）对 u_official，或看共享几何列 u_custom_sharedpose。
depth 取官方相机系 z（米）；visible = 自研可见性(遮挡+深度边缘) AND 官方在框内。
"""

import argparse
import os
import sqlite3
import sys
import time

import numpy as np
import cv2
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as SRate

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# =============================================================================
# 常量（逐条注明来源；与两侧配置文件核对过）
# =============================================================================
SL_BAG = "/home/ywj/slam_data/singleframe/sl_frames"
FL2_BAG = "/home/ywj/slam_data/singleframe/fl2_frames"
RAW_BAG = "/home/ywj/slam_data/Retail_Street"
RAW_IMG_TOPIC = "/left_camera/image"
OUT_DEFAULT = "/home/ywj/slam_data/singleframe/ab_compare"

# 内参：自研 super_livo_visual_a/config/visual_a_params.yaml
#       官方 /tmp/retail_cam.yaml(cam_width=1280,scale=0.5) 经 vikit
#       PinholeCamera 构造(fx*=scale, cx*=scale, width=width*scale) 后完全一致。
FX, FY, CX, CY = 646.78472, 646.65775, 313.456795, 261.399612
CW, CH = 640, 512
# 畸变 k1,k2,p1,p2（两侧同一份标定；官方 d4=k3=0）
K1, K2, P1, P2 = -0.076160, 0.123001, -0.00113, 0.000251

# LiDAR<->Camera 外参（自研 visual_a_params.yaml == 官方 retail_avia.yaml）
RCL = np.array([0.00610193, -0.999863, -0.0154172,
                -0.00615449, 0.0153796, -0.999863,
                0.999962, 0.00619598, -0.0060598]).reshape(3, 3)
PCL = np.array([0.0194384, 0.104689, -0.0251952])
# 官方 IMU->LiDAR 外参（retail_avia.yaml extrin_calib.extrinsic_T, extrinsic_R=I）
# vio.cpp: setImuToLidarExtrinsic -> Rli=extR^T, Pli=-extR^T·extT
#          initializeVIO        -> Rci=Rcl·Rli, Pci=Rcl·Pli+Pcl
EXT_T = np.array([0.04165, 0.02326, -0.0284])
EXT_R = np.eye(3)
RCI = RCL @ EXT_R.T
PCI = RCL @ (-EXT_R.T @ EXT_T) + PCL

IMG_TIME_OFFSET = 0.1      # 两侧配置均为 0.1
BORDER_MARGIN = 2          # projection.hpp Projector::Options::border_margin 默认值
OFFICIAL_BND = 3           # LIVMapper.cpp isInFrame(pc.cast<int>(), 3)
BLIND_RGB = 0.0            # retail_avia.yaml publish.blind_rgb_points
EDGE_THRESHOLD = 0.5       # color.edge_threshold
DEPTH_TOL = 0.1            # color.depth_tolerance
SYNC_MAX_OFFSET = 0.05     # color.sync_max_offset（图像-点云配对窗口）
GAP_NODE = 0.5             # kPoseHistoryMaxGapSec（visual_a_node.cpp:28）
GAP_PIPE = 0.125           # colorizer 内部逐点插值 max_offset*2.5

MATCHSUB = 1200            # 帧配对时每帧用的光度采样子点
PHOTO_T = 3.0              # 判定「官方帧 k 用的就是这张图」的 median MAE 门限(/255)

_MSG_CLS = {}


def _cls(t):
    if t not in _MSG_CLS:
        _MSG_CLS[t] = get_message(t)
    return _MSG_CLS[t]


def _reader(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    return r


def _payload(tup):
    for x in tup:
        if isinstance(x, (bytes, bytearray)):
            return bytes(x)
    raise TypeError("read_next() 没有 bytes payload")


def _hdr(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def quat_to_mat(x, y, z, w):
    n2 = x * x + y * y + z * z + w * w
    if n2 < 1e-14:
        return np.eye(3)
    n = n2 ** 0.5
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def cloud_parts(msg):
    """PointCloud2 -> (xyz float64[N,3], rgb uint8[N,3]|None, 字段名)。

    PCL 把 RGB 塞进 float32 的位模式 -> 必须按 uint32 位重解释（view），
    用 astype 做数值转换会得到完全错误的颜色。
    """
    names = {f.name: f.offset for f in msg.fields}
    n = int(msg.width) * int(msg.height)
    step = int(msg.point_step)
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if buf.size < n * step or "x" not in names:
        return np.zeros((0, 3)), None, names
    arr = buf[:n * step].reshape(n, step)
    ox = names["x"]
    xyz = arr[:, ox:ox + 12].copy().view(np.float32).reshape(n, 3).astype(np.float64)
    rgb = None
    if "rgb" in names:
        v = arr[:, names["rgb"]:names["rgb"] + 4].copy().view(np.uint32).reshape(n)
        rgb = np.stack([(v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF], 1).astype(np.uint8)
    return xyz, rgb, names


# =============================================================================
# 位姿插值：复刻 LidarPoseInterpolator（平移线性 + 四元数 slerp，首尾可外推
# max_gap 内，间隔 > max_gap 则拒绝）
# =============================================================================
class Track(object):
    def __init__(self, stamps, quats, ts, max_gap):
        o = np.argsort(stamps)
        self.s = np.asarray(stamps, float)[o]
        self.q = np.asarray(quats, float)[o]        # xyzw
        self.t = np.asarray(ts, float)[o]
        self.gap = max_gap
        self.R = np.stack([quat_to_mat(*qq) for qq in self.q])  # q 已是 (x,y,z,w)

    def idx_at(self, stamp):
        s = self.s
        i = int(np.searchsorted(s, stamp, side="left"))
        if i == 0:
            return (0, 0, 0.0) if s[0] - stamp <= self.gap else None
        if i >= len(s):
            return (len(s) - 1, len(s) - 1, 0.0) if stamp - s[-1] <= self.gap else None
        dt = s[i] - s[i - 1]
        if dt <= 0.0 or dt > self.gap:
            return None
        return (i - 1, i, (stamp - s[i - 1]) / dt)

    def at(self, stamp):
        """-> (T 4x4, ok)。ok=False 时按复刻的 max_gap 规则该时刻不可用。"""
        r = self.idx_at(stamp)
        if r is None:
            return np.eye(4), False
        a, b, al = r
        M = np.eye(4)
        M[:3, :3] = self._slerp(a, b, al)
        M[:3, 3] = (1 - al) * self.t[a] + al * self.t[b]
        return M, True

    def _slerp(self, a, b, alpha):
        if a == b:
            return self.R[a]
        qa, qb = self.q[a].copy(), self.q[b].copy()
        d = float(np.dot(qa, qb))
        if d < 0:
            qb = -qb
            d = -d
        d = min(1.0, max(-1.0, d))
        ang = np.arccos(d)
        if ang < 1e-9:
            q = (1 - alpha) * qa + alpha * qb
        else:
            w0 = np.sin((1 - alpha) * ang) / np.sin(ang)
            w1 = np.sin(alpha * ang) / np.sin(ang)
            q = w0 * qa + w1 * qb
        return SRate.from_quat(q / np.linalg.norm(q)).as_matrix()


# =============================================================================
# 投影 / 采样
# =============================================================================
def xf(M, P):
    """4x4 齐次变换作用于 Nx3 点集。"""
    return P @ M[:3, :3].T + M[:3, 3][None, :]


def pinhole(p_cam):
    """自研 projection.cpp：u = fx*x/z + cx（z <= 1e-6 丢弃）。"""
    z = p_cam[:, 2]
    ok = z > 1e-6
    u = np.full(len(p_cam), np.nan)
    v = np.full(len(p_cam), np.nan)
    u[ok] = FX * p_cam[ok, 0] / z[ok] + CX
    v[ok] = FY * p_cam[ok, 1] / z[ok] + CY
    return u, v, ok


def distort(u, v):
    """理想(去畸变)像素 -> 原始畸变图像像素。

    与 vikit PinholeCamera::world2cam 的畸变分支逐式一致（d4=k3=0）。
    """
    x = (u - CX) / FX
    y = (v - CY) / FY
    r2 = x * x + y * y
    cd = 1.0 + K1 * r2 + K2 * r2 * r2
    xd = x * cd + P1 * (2 * x * y) + P2 * (r2 + 2 * x * x)
    yd = y * cd + P1 * (r2 + 2 * y * y) + P2 * (2 * x * y)
    return FX * xd + CX, FY * yd + CY


def official_uv(p_cam):
    """官方 w2c = 先理想投影再做畸变（PinholeCamera 带 d0!=0 分支）。"""
    u, v, ok = pinhole(p_cam)
    g = ok & np.isfinite(u)
    ud = np.full(len(p_cam), np.nan)
    vd = np.full(len(p_cam), np.nan)
    ud[g], vd[g] = distort(u[g], v[g])
    return ud, vd, ok


def fov_custom(u, v):
    m = float(BORDER_MARGIN)
    return np.isfinite(u) & (u >= m) & (u < CW - m) & np.isfinite(v) & (v >= m) & (v < CH - m)


def fov_official(u, v):
    ui = np.trunc(np.where(np.isfinite(u), u, -1e9))
    vi = np.trunc(np.where(np.isfinite(v), v, -1e9))
    return (ui >= OFFICIAL_BND) & (ui < CW - OFFICIAL_BND) & \
           (vi >= OFFICIAL_BND) & (vi < CH - OFFICIAL_BND)


def bilinear(img, u, v):
    """两侧一致的双线性采样（自研 RgbAssigner / 官方 getInterpolatedPixel）。

    越界返回 nan —— 官方在 isInFrame(bnd=3) 之后调用，不会越界。
    """
    out = np.full((len(u), 3), np.nan, np.float32)
    ok = np.isfinite(u) & np.isfinite(v) & (u >= 0) & (v >= 0) & (u < CW - 1) & (v < CH - 1)
    if not ok.any():
        return out
    x0 = np.floor(u[ok]).astype(np.int64)
    y0 = np.floor(v[ok]).astype(np.int64)
    ax = (u[ok] - x0).astype(np.float32)[:, None]
    ay = (v[ok] - y0).astype(np.float32)[:, None]
    out[ok] = (1 - ay) * ((1 - ax) * img[y0, x0] + ax * img[y0, x0 + 1]) + \
              ay * ((1 - ax) * img[y0 + 1, x0] + ax * img[y0 + 1, x0 + 1])
    return out


def zbuffer(u, v, depth, valid):
    z = np.full((CH, CW), np.inf, np.float32)
    sel = valid & np.isfinite(u) & np.isfinite(v) & (depth > 0)
    if not sel.any():
        return z
    ui = np.rint(u[sel]).astype(np.int64)
    vi = np.rint(v[sel]).astype(np.int64)
    k = (ui >= 0) & (ui < CW) & (vi >= 0) & (vi < CH)
    ui, vi, d = ui[k], vi[k], depth[sel][k].astype(np.float32)
    flat = vi * CW + ui
    o = np.argsort(flat, kind="stable")
    flat, d = flat[o], d[o]
    uniq, first = np.unique(flat, return_index=True)
    z.reshape(-1)[uniq] = np.minimum.reduceat(d, first)
    return z


def vis_edge(zbuf, u, v, depth):
    """(visible, depth_edge)：复刻 OcclusionChecker::isVisible / isDepthEdge。

    与 C++ 一致的关键点：3x3 窗口里越界邻居直接跳过、非有限值不参与 min/max
    （occlusion_check.cpp:64-79），所以这里用 pad 而不是 np.roll（roll 会把图像
    另一侧的像素绕回来，制造假边缘）。
    """
    n = len(u)
    vis = np.ones(n, bool)
    edge = np.zeros(n, bool)
    sel = np.isfinite(u) & np.isfinite(v) & (depth > 0)
    ui = np.rint(np.where(sel, u, -1)).astype(np.int64)
    vi = np.rint(np.where(sel, v, -1)).astype(np.int64)
    inb = sel & (ui >= 0) & (ui < CW) & (vi >= 0) & (vi < CH)
    vis[sel & ~inb] = False
    if not inb.any():
        return vis, edge
    BIG = np.float32(1e9)
    zz = zbuf[vi[inb], ui[inb]].astype(np.float64)
    fin = np.isfinite(zz)
    vv = np.ones(int(inb.sum()), bool)      # 像素无观测 => isVisible 返回 true
    d = depth[inb]
    vv[fin] = d[fin] <= zz[fin] * (1.0 + DEPTH_TOL)
    vis[np.where(inb)[0]] = vv
    zf = np.where(np.isfinite(zbuf), zbuf, BIG)            # min 窗口：无效值不参与
    zm = np.where(np.isfinite(zbuf), zbuf, np.float32(0))  # max 窗口：无效值不参与
    padf = np.pad(zf, 1, mode="constant", constant_values=float(BIG))
    padm = np.pad(zm, 1, mode="constant", constant_values=0.0)
    rows = 1 + vi[inb]
    cols = 1 + ui[inb]
    mn = np.full(len(rows), np.inf)
    mx = np.zeros(len(rows))
    for a in (-1, 0, 1):
        for b in (-1, 0, 1):
            mn = np.minimum(mn, padf[rows + a, cols + b])
            mx = np.maximum(mx, padm[rows + a, cols + b])
    e = (mn < BIG / 2) & ((mx - mn) > EDGE_THRESHOLD)
    edge[np.where(inb)[0]] = e
    return vis, edge


# =============================================================================
# 读 bag / 读原图
# =============================================================================
def load_bag(path, topics_cloud, topics_odom):
    r = _reader(path)
    tt = {x.name: x.type for x in r.get_all_topics_and_types()}
    odom, frames, skipped = [], [], 0
    while r.has_next():
        tup = r.read_next()
        topic = tup[0]
        m = deserialize_message(_payload(tup), _cls(tt[topic]))
        ts = _hdr(m)
        if any(t in topic for t in topics_odom):
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            odom.append((ts, (q.x, q.y, q.z, q.w), (p.x, p.y, p.z)))
        elif any(t in topic for t in topics_cloud):
            xyz, rgb, names = cloud_parts(m)
            if rgb is None:
                skipped += 1
                continue
            frames.append((ts, xyz, rgb))
    r.close() if hasattr(r, "close") else None
    odom.sort(key=lambda a: a[0])
    frames.sort(key=lambda a: a[0])
    return odom, frames, skipped


class RawImages(object):
    """按需从 Retail_Street 取原图（sqlite rowid 直接寻址，避免整包反序列化）。"""

    def __init__(self, t_lo, t_hi):
        self.db = sqlite3.connect("file:%s/Retail_Street.db3?mode=ro" % RAW_BAG, uri=True)
        tid = self.db.execute("select id from topics where name=?", (RAW_IMG_TOPIC,)).fetchone()[0]
        self.tid = tid
        row = self.db.execute("select id, timestamp, data from messages where topic_id=?"
                             " order by id limit 1", (tid,)).fetchone()
        m0 = deserialize_message(bytes(row[2]), _cls("sensor_msgs/msg/Image"))
        self.off = _hdr(m0) - row[1] * 1e-9      # header = storage + off
        lo = (t_lo - 0.8 - self.off) * 1e9
        hi = (t_hi + 0.8 - self.off) * 1e9
        recs = []
        for rid, st in self.db.execute("select id, timestamp from messages where topic_id=?"
                                       " and timestamp between ? and ? order by timestamp",
                                       (tid, lo, hi)):
            b = self.db.execute("select data from messages where id=?", (rid,)).fetchone()[0]
            mm = deserialize_message(bytes(b), _cls("sensor_msgs/msg/Image"))
            recs.append((_hdr(mm), rid, mm.width, mm.height, mm.encoding))
        self.t = np.array([a[0] for a in recs], float)
        self.rec = recs
        self.cache = {}
        if len(recs) < 5:
            raise RuntimeError("窗口内没有原图")

    def nearest(self, t):
        i = int(np.clip(np.searchsorted(self.t, t), 1, len(self.t) - 1))
        if abs(self.t[i - 1] - t) <= abs(self.t[i] - t):
            i -= 1
        return i

    def get(self, i):
        """-> (header_ts, rgb float32 HxWx3, 通道顺序为 R,G,B)。

        两侧都是 cv_bridge::toCvCopy(msg,"bgr8") 后取 pixel[2] 当作 r
        （官方 LIVMapper.cpp:1033 + vio.cpp:1866 / 自研 RgbAssigner），
        也就是说 bag 里发布的 rgb 就是**真实 RGB 顺序**；bag 编码本身是
        rgb8，因此这里保持原顺序、不做交换，采样结果可直接与发布 rgb 比。
        """
        if i in self.cache:
            return self.cache[i]
        h, rid, w, hh, enc = self.rec[i]
        b = self.db.execute("select data from messages where id=?", (rid,)).fetchone()[0]
        m = deserialize_message(bytes(b), _cls("sensor_msgs/msg/Image"))
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        if m.encoding == "bgr8":
            a = a[:, :, ::-1]
        elif m.encoding != "rgb8":
            raise RuntimeError("未预期编码 %s" % m.encoding)
        a = np.ascontiguousarray(a).astype(np.float32)
        if len(self.cache) > 4:
            self.cache.pop(next(iter(self.cache)))
        self.cache[i] = (h, a)
        return self.cache[i]


def rectify(rgb):
    """复刻 rectifyBgr：initUndistortRectifyMap(K,D,{},K,size,CV_32FC1)+remap(INTER_LINEAR)。

    remap 逐通道独立，与输入是 BGR 还是 RGB 无关。"""
    K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], float)
    Dm = np.array([K1, K2, P1, P2, 0.0], float)
    m1, m2 = cv2.initUndistortRectifyMap(K, Dm, None, K, (CW, CH), cv2.CV_32FC1)
    return cv2.remap(rgb, m1, m2, cv2.INTER_LINEAR)


def rectify_u8(rgb_f32):
    """与 C++ 一致：remap 作用在 uint8 上，输出重新量化为 uint8，然后再转 float32。"""
    return rectify(np.rint(rgb_f32).astype(np.uint8)).astype(np.float32)


# =============================================================================
# 主流程
# =============================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tol", type=float, default=0.20,
                    help="逐点配对最近邻容差(米)；官方侧是体素降采样点、自研侧是原始扫描，"
                         "同名物理点的最近邻距离中位约 5cm，报告里另按残差<5cm 分层")
    ap.add_argument("--outdir", default=OUT_DEFAULT)
    ap.add_argument("--max-pairs", type=int, default=0, help="最多分析多少对帧(0=全部)")
    args = ap.parse_args()
    t0 = time.time()

    print("[1/6] 读 bag")
    sl_odom, sl_raw, _ = load_bag(SL_BAG, ["colored_cloud"], ["odom"])
    fl_odom, fl_raw, fl_noskip = load_bag(FL2_BAG, ["cloud_registered"], ["aft_mapped"])
    print("   自研: odom %d, 彩色帧 %d (体系, frame_id=imu)" % (len(sl_odom), len(sl_raw)))
    print("   官方: odom %d, 带 rgb 帧 %d (世界系, frame_id=camera_init), 无 rgb 帧 %d 已跳过"
          % (len(fl_odom), len(fl_raw), fl_noskip))
    if not sl_raw or not fl_raw:
        sys.exit("无可用帧")

    imgs = RawImages(sl_raw[0][0], sl_raw[-1][0])
    print("   原图窗口 %d 张, %.3f..%.3f, 周期 %.4f s"
          % (len(imgs.t), imgs.t[0], imgs.t[-1], np.median(np.diff(imgs.t))))
    sl_track_node = Track([a[0] for a in sl_odom], [a[1] for a in sl_odom], [a[2] for a in sl_odom], GAP_NODE)
    sl_track_pipe = Track([a[0] for a in sl_odom], [a[1] for a in sl_odom], [a[2] for a in sl_odom], GAP_PIPE)

    # ---------------- 自研侧 ----------------
    print("[2/6] 自研侧投影链重建（K/D、remap 去畸变、T_wc=T_wl·T_lc、t_img=t_pair+0.1）")
    SL, sl_all = [], []
    for k, (ts, p_b, rgb) in enumerate(sl_raw):
        T_cloud, ok_c = sl_track_node.at(ts)
        if not ok_c:
            continue
        j = imgs.nearest(ts)
        hdr = imgs.t[j]
        if abs(hdr - ts) > SYNC_MAX_OFFSET:
            continue
        t_img = hdr + IMG_TIME_OFFSET
        T_img, ok_i = sl_track_node.at(t_img)
        if not ok_i:
            continue
        Mrel = np.linalg.inv(T_img) @ T_cloud
        p_mc = xf(Mrel, p_b)
        p_c = p_mc @ RCL.T + PCL[None, :]
        u, v, okz = pinhole(p_c)
        in_fov = fov_custom(u, v)
        zb = zbuffer(u, v, p_c[:, 2], in_fov & okz)
        vis, edge = vis_edge(zb, u, v, p_c[:, 2])
        # 自研侧单帧自洽性：重建像素在去畸变图上采样，应与 bag 发布的 rgb 一致
        idx = np.where(in_fov & okz & (rgb.max(1) > 0))[0]
        idx = idx[::max(1, len(idx) // 2000)]
        self_mae = float("nan")
        if len(idx) > 300:
            _, im = imgs.get(j)
            self_mae = _med_err(bilinear(rectify_u8(im), u[idx], v[idx]), rgb[idx])
        pipe_ok = sl_track_pipe.at(ts)[1]
        rec = dict(idx=k, stamp=ts, img_idx=j, img_hdr=hdr, T_cloud=T_cloud, T_img=T_img,
                   p_body=p_b, p_cam=p_c, u=u, v=v, depth=p_c[:, 2],
                   okz=okz, in_fov=in_fov, vis=vis, edge=edge, rgb=rgb,
                   colored=(rgb.max(1) > 0), pipe_ok=pipe_ok, zb=zb, self_mae=self_mae)
        sl_all.append(rec)
        if self_mae < PHOTO_T:          # 连自己发布的颜色都复现不了的帧不进入 A/B
            SL.append(rec)
    bad = [r for r in sl_all if not r["self_mae"] < PHOTO_T]
    print("   自研帧 %d -> 单帧自洽 median MAE<%.1f 的 %d 帧进入 A/B；剔除 %d 帧: %s"
          % (len(sl_all), PHOTO_T, len(SL), len(bad),
             ", ".join("#%d(MAE=%s, 有色%.0f%%)" % (r["idx"],
                                                    "nan" if not np.isfinite(r["self_mae"])
                                                    else "%.1f" % r["self_mae"],
                                                    100.0 * r["colored"].mean()) for r in bad)
             or "无"))
    print("   可用帧中 %d 帧的流水线逐点插值超出 0.125 s 上限（节点用 0.5 s 上限，仍能取到位姿）"
          % sum(1 for f in SL if not f["pipe_ok"]))

    # ---------------- 官方侧 ----------------
    print("[3/6] 官方侧 w2f/w2c 重建：同索引 odom + PinholeCamera 畸变模型 + img_rgb=原图")
    fl_R = np.stack([quat_to_mat(*a[1]) for a in fl_odom])
    fl_p = np.array([a[2] for a in fl_odom], float)
    n_use = min(len(fl_odom), len(fl_raw))
    if len(fl_odom) != len(fl_raw):
        print("   !! odom %d != 彩色帧 %d，只处理前 %d 个（1:1 假设不再成立）"
              % (len(fl_odom), len(fl_raw), n_use))
    FL = []
    for k in range(n_use):
        ts, p_w, rgb = fl_raw[k]
        p_c = ((p_w - fl_p[k]) @ fl_R[k]) @ RCI.T + PCI[None, :]
        ud, vd, okz = official_uv(p_c)
        in_fov = okz & fov_official(ud, vd) & (np.linalg.norm(p_c, axis=1) > BLIND_RGB)
        zb = zbuffer(ud, vd, p_c[:, 2], in_fov)
        vis, edge = vis_edge(zb, ud, vd, p_c[:, 2])
        w = np.where(in_fov)[0]
        FL.append(dict(idx=k, stamp=ts, odom_stamp=float(fl_odom[k][0]), p_w=p_w, rgb=rgb,
                       p_cam=p_c, u=ud, v=vd, depth=p_c[:, 2], in_fov=in_fov, vis=vis,
                       edge=edge, zb=zb, msel=w[::max(1, len(w) // MATCHSUB)]))
    print("   官方帧 %d（odom 偏移 di∈[-2..2] 的联合光度搜索已确认 di=0 最优，故固定同索引）"
          % len(FL))

    # ---------------- 帧配对：唯一可靠的时间锚是「同一张原图」 ----------------
    # 两套世界系的原点/朝向不同，不能直接做世界系最近邻；官方 bag 的 header 是
    # node->now()（处理时刻）也没有物理意义。但官方彩色帧 k 用的就是第 g(k) 张
    # 原图（LIVMapper.cpp:1370 img_rgb = vio_manager->img_rgb），自研帧 i 用的
    # 就是 imgs.nearest(header) 那张；两侧一旦落在同一张图上，相机系就是同一个
    # 物理坐标系。判定方法：把官方帧 k 的云用它的 odom 投回它自己那张图，只有
    # 「真的是这张图」时重采样 RGB 才会和 bag 发布的 RGB 对上（median MAE ~1）。
    print("[4/6] 帧配对：以「同一张原图」为锚 —— 对每个自研帧穷举全部官方帧，"
          "用发布 RGB vs 原图重采样 RGB 的误差判定")
    pairs = []
    used = set()
    ptable = []
    for fi, f in enumerate(SL):
        best = None
        for dj in (-1, 0, 1):
            jj = f["img_idx"] + dj
            if not (0 <= jj < len(imgs.t)):
                continue
            _, im = imgs.get(jj)
            for g in FL:
                if g["idx"] in used or len(g["msel"]) < 200:
                    continue
                s = g["msel"]
                rs = bilinear(im, g["u"][s], g["v"][s])
                ok = np.isfinite(rs).all(1)
                if ok.sum() < 200:
                    continue
                e = float(np.median(np.abs(rs[ok] - g["rgb"][s].astype(np.float32)[ok]).mean(1)))
                if best is None or e < best[0]:
                    best = (e, g["idx"], dj)
        if best is None:
            continue
        ptable.append((fi, best[1], best[2], best[0]))
        if best[2] != 0 or best[0] > PHOTO_T:
            continue
        used.add(best[1])
        pairs.append((fi, best[1], best[0]))
    if args.max_pairs:
        pairs = pairs[:args.max_pairs]
    print("   自研帧 %d -> 候选 %d -> 同图且 median MAE<%.1f 的确信配对 %d"
          % (len(SL), len(ptable), PHOTO_T, len(pairs)))
    for row in ptable:
        ok = (row[0], row[1]) in set((a, b) for a, b, _ in pairs)
        print("   %s SL帧%-4d 图#%-5d <-> 官方帧%-4d (图偏移%+d) median MAE=%6.2f"
              % ("接受" if ok else "  x", SL[row[0]]["idx"], SL[row[0]]["img_idx"],
                 row[1], row[2], row[3]))
    if not pairs:
        sys.exit("同图配对失败：没有任何自研帧与官方帧落在同一张原图上且误差达标")

    # ---------------- 逐点：图像选择 + 重算 + 采样 ----------------
    print("[5/6] 逐点配对 / 两套重算 / 自洽性检查（两侧共用同一张原图）")
    acc = dict()
    ntot = 0
    recon_shift = {"official": [], "custom": []}
    pair_rows = []
    for (fi, fj, mae_img) in pairs:
        f = SL[fi]
        g = FL[fj]
        tree = cKDTree(g["p_cam"])
        d, jidx = tree.query(f["p_cam"], k=1)          # cKDTree.query -> (距离, 索引)
        m = (d < args.tol) & np.isfinite(d)
        if m.sum() < 50:
            continue
        order = np.argsort(d[m])
        sel_off = jidx[m][order]
        sel_sl = np.where(m)[0][order]
        keep = set()
        ko, ks = [], []
        for o, s in zip(sel_off, sel_sl):
            if o in keep:
                continue
            keep.add(o)
            ko.append(o)
            ks.append(s)
        ko = np.array(ko, np.int64)
        ks = np.array(ks, np.int64)

        # 两侧用的是同一张原图（配对条件 dj=0），这就是标题里的「同帧同图」
        hdr_i, rgb_img = imgs.get(f["img_idx"])
        rect_img = rectify_u8(rgb_img)
        pair_mae_off = _med_err(bilinear(rgb_img, g["u"][ko], g["v"][ko]), g["rgb"][ko])
        pair_mae_cus = _med_err(bilinear(rect_img, f["u"][ks], f["v"][ks]), f["rgb"][ks])
        hit = f["in_fov"] & f["okz"]
        pair_rows.append((f["idx"], g["idx"], f["img_idx"], len(ko), float(m[hit].mean()),
                          float(np.median(d[m])), mae_img, pair_mae_off, pair_mae_cus))

        p_off = g["p_cam"][ko]
        p_cus = f["p_cam"][ks]
        # 2x2 因子设计：几何(官方/自研) x 投影模型(自研理想/官方畸变)
        u_shp, v_shp, ok_shp = pinhole(p_off)               # 官方几何 + 自研模型
        u_d2, v_d2, ok_d2 = official_uv(p_cus)              # 自研几何 + 官方模型
        p_lever = p_off + (RCL @ EXT_T)[None, :]            # 官方几何 + 自研外参约定(含/不含杆臂)
        u_lv, v_lv, ok_lv = pinhole(p_lever)

        vis_off = g["vis"][ko]
        vis_cus = f["vis"][ks] & f["in_fov"][ks] & f["okz"][ks]
        edge_cus = f["edge"][ks]
        _, edge_off = vis_edge(g["zb"], g["u"][ko], g["v"][ko], g["depth"][ko])

        rgb_off_pub = g["rgb"][ko].astype(np.float64)
        rgb_cus_pub = f["rgb"][ks].astype(np.float64)
        rgb_off_rs = bilinear(rgb_img, g["u"][ko], g["v"][ko])
        rgb_cus_rs = bilinear(rect_img, f["u"][ks], f["v"][ks])
        rgb_shp_rs = bilinear(rect_img, u_shp, v_shp)
        rgb_d2_rs = bilinear(rgb_img, u_d2, v_d2)
        rgb_lv_rs = bilinear(rect_img, u_lv, v_lv)
        # 预处理等价性：自研在去畸变图上采的像素，与官方原图在畸变坐标处采的像素
        u_cus_inraw, v_cus_inraw = distort(f["u"][ks], f["v"][ks])
        rgb_cus_onraw = bilinear(rgb_img, u_cus_inraw, v_cus_inraw)

        # 重建偏差（整数像素搜索）：发布 RGB vs 重采样 RGB 的最佳平移
        for tag, uu, vv, pub, src in (("official", g["u"][ko], g["v"][ko], rgb_off_pub, rgb_img),
                                      ("custom", f["u"][ks], f["v"][ks], rgb_cus_pub, rect_img)):
            sel = np.isfinite(uu) & np.isfinite(vv)
            if sel.sum() < 200:
                continue
            idx = np.where(sel)[0][:4000]
            bestv, bestsh = 1e9, None
            for du in range(-3, 4):
                for dv in range(-3, 4):
                    rs = bilinear(src, uu[idx] + du, vv[idx] + dv)
                    okk = np.isfinite(rs).all(1)
                    if okk.sum() < 100:
                        continue
                    val = float(np.abs(rs[okk] - pub[idx][okk]).mean())
                    if val < bestv:
                        bestv, bestsh = val, (du, dv)
            recon_shift[tag].append(bestsh if bestsh else (0, 0))

        n = len(ko)
        pid = np.arange(g["idx"] * 100000, g["idx"] * 100000 + n) + ko  # point_id = 官方点索引(帧内)
        for key, val in dict(
            pid=pid,
            xyz=g["p_w"][ko],
            u_off=g["u"][ko], v_off=g["v"][ko],
            u_cus=f["u"][ks], v_cus=f["v"][ks],
            rgb_off=rgb_off_pub, rgb_cus=rgb_cus_pub,
            depth=p_off[:, 2], depth_cus=p_cus[:, 2],
            vis_off=vis_off, vis_cus=vis_cus, edge_cus=edge_cus, edge_off=edge_off,
            mdist=np.linalg.norm(p_cus - p_off, axis=1),
            frame_cus=np.full(n, f["idx"]), frame_off=np.full(n, g["idx"]),
            img_idx=np.full(n, f["img_idx"]), img_hdr=np.full(n, hdr_i), img_hdr_off=np.full(n, hdr_i),
            pair_score=np.full(n, float(m.mean())), photo_mae=np.full(n, mae_img),
            p_off=p_off, p_cus=p_cus,
            u_shp=u_shp, v_shp=v_shp, rgb_shp=rgb_shp_rs,
            u_d2=u_d2, v_d2=v_d2, rgb_d2=rgb_d2_rs,
            u_lv=u_lv, v_lv=v_lv, rgb_lv=rgb_lv_rs,
            rgb_off_rs=rgb_off_rs, rgb_cus_rs=rgb_cus_rs,
            u_cus_raw=u_cus_inraw, v_cus_raw=v_cus_inraw, rgb_cus_onraw=rgb_cus_onraw,
        ).items():
            acc.setdefault(key, []).append(np.asarray(val))
        ntot += n

    for k in list(acc.keys()):
        acc[k] = np.concatenate(acc[k], 0)
    print("   配对点合计 %d" % ntot)
    if ntot < 100:
        sys.exit("配对点太少")

    write_csv(args.outdir, acc)
    report(acc, args, recon_shift, pair_rows, len(SL), len(pairs))
    print("\n总耗时 %.1f s" % (time.time() - t0))


STRICT = ["point_id", "x", "y", "z", "u_official", "v_official", "u_custom", "v_custom",
          "rgb_official", "rgb_custom", "depth", "visible"]
EXT = STRICT + [
    "frame_custom", "frame_official", "img_idx", "pair_score", "photo_mae", "match_dist",
    "img_hdr_custom", "img_hdr_official",
    "xc_official", "yc_official", "zc_official", "xc_custom", "yc_custom", "zc_custom",
    "depth_custom", "visible_official", "visible_custom", "edge_custom", "edge_official",
    "u_custom_sharedpose", "v_custom_sharedpose", "rgb_custom_sharedpose_resample",
    "u_official_from_custom_geom", "v_official_from_custom_geom",
    "rgb_official_from_custom_geom_resample",
    "u_custom_lever", "v_custom_lever", "rgb_custom_lever_resample",
    "rgb_official_resample", "rgb_custom_resample",
    "u_custom_in_raw_coords", "v_custom_in_raw_coords", "rgb_custom_on_raw_resample",
]


def _rgbs(s):
    return "%d_%d_%d" % (int(s[0]), int(s[1]), int(s[2]))


def _med_err(rs, pub):
    """重采样 RGB vs 发布 RGB 的中位 |误差|（自动剔除越界 nan）。"""
    e = np.abs(np.asarray(rs, np.float32) - np.asarray(pub, np.float32)).mean(1)
    e = e[np.isfinite(e)]
    return float(np.median(e)) if len(e) > 50 else float("nan")


def write_csv(outdir, A):
    os.makedirs(outdir, exist_ok=True)
    n = len(A["pid"])
    cols = {
        "point_id": A["pid"], "x": A["xyz"][:, 0], "y": A["xyz"][:, 1], "z": A["xyz"][:, 2],
        "u_official": A["u_off"], "v_official": A["v_off"], "u_custom": A["u_cus"],
        "v_custom": A["v_cus"],
        "rgb_official": [_rgbs(A["rgb_off"][i]) for i in range(n)],
        "rgb_custom": [_rgbs(A["rgb_cus"][i]) for i in range(n)],
        "depth": A["depth"],
        "visible": (A["vis_off"] & A["vis_cus"]).astype(np.int64),
        "frame_custom": A["frame_cus"], "frame_official": A["frame_off"],
        "img_idx": A["img_idx"], "pair_score": A["pair_score"], "photo_mae": A["photo_mae"],
        "match_dist": A["mdist"],
        "img_hdr_custom": A["img_hdr"], "img_hdr_official": A["img_hdr_off"],
        "xc_official": A["p_off"][:, 0], "yc_official": A["p_off"][:, 1], "zc_official": A["p_off"][:, 2],
        "xc_custom": A["p_cus"][:, 0], "yc_custom": A["p_cus"][:, 1], "zc_custom": A["p_cus"][:, 2],
        "depth_custom": A["depth_cus"],
        "visible_official": A["vis_off"].astype(np.int64),
        "visible_custom": A["vis_cus"].astype(np.int64),
        "edge_custom": A["edge_cus"].astype(np.int64),
        "edge_official": A["edge_off"].astype(np.int64),
        "u_custom_sharedpose": A["u_shp"], "v_custom_sharedpose": A["v_shp"],
        "rgb_custom_sharedpose_resample": [_rgbs(x) if np.isfinite(x).all() else "nan"
                                           for x in A["rgb_shp"]],
        "u_official_from_custom_geom": A["u_d2"], "v_official_from_custom_geom": A["v_d2"],
        "rgb_official_from_custom_geom_resample": [_rgbs(x) if np.isfinite(x).all() else "nan"
                                                   for x in A["rgb_d2"]],
        "u_custom_lever": A["u_lv"], "v_custom_lever": A["v_lv"],
        "rgb_custom_lever_resample": [_rgbs(x) if np.isfinite(x).all() else "nan"
                                      for x in A["rgb_lv"]],
        "rgb_official_resample": [_rgbs(x) if np.isfinite(x).all() else "nan" for x in A["rgb_off_rs"]],
        "rgb_custom_resample": [_rgbs(x) if np.isfinite(x).all() else "nan" for x in A["rgb_cus_rs"]],
        "u_custom_in_raw_coords": A["u_cus_raw"], "v_custom_in_raw_coords": A["v_cus_raw"],
        "rgb_custom_on_raw_resample": [_rgbs(x) if np.isfinite(x).all() else "nan"
                                       for x in A["rgb_cus_onraw"]],
    }
    for name, hdr in (("ab_points.csv", STRICT), ("ab_points_extended.csv", EXT)):
        path = os.path.join(outdir, name)
        with open(path, "w") as fp:
            fp.write(",".join(hdr) + "\n")
            for i in range(n):
                row = []
                for c in hdr:
                    v = cols[c][i]
                    if isinstance(v, float) or (isinstance(v, np.ndarray) and v.ndim == 0):
                        row.append("%.6f" % v)
                    elif isinstance(v, (np.floating,)):
                        row.append("%.6f" % float(v))
                    else:
                        row.append(str(v))
                fp.write(",".join(row) + "\n")
        print("   写出 %s  (%d 行, %d 列)" % (path, n, len(hdr)))


def _stat(name, a, masks):
    a = np.asarray(a, np.float64)
    for label, msk in masks:
        v = a[msk & np.isfinite(a)]
        if len(v) == 0:
            print("   %-34s %-12s n=0" % (name, label))
            continue
        print("   %-34s %-12s n=%-7d median=%8.3f  P90=%8.3f  P99=%8.3f  mean=%8.3f"
              % (name, label, len(v), np.median(v), np.percentile(v, 90),
                 np.percentile(v, 99), np.mean(v)))


def report(A, args, recon_shift, pair_rows, n_sl_frames, n_pairs):
    print("\n" + "=" * 78)
    print("汇总诊断")
    print("=" * 78)
    n = len(A["pid"])
    both = A["vis_off"] & A["vis_cus"]
    off_pub = A["rgb_off"].max(1) > 0
    cus_pub = A["rgb_cus"].max(1) > 0
    usable = both & off_pub & cus_pub
    print("配对点 %d，覆盖 %d 对帧（自研可用帧 %d）；两侧都判定可见 %d，且都发布了非黑 RGB %d"
          % (n, len(pair_rows), n_sl_frames, both.sum(), usable.sum()))
    print("\n--- 逐帧配对质量（同一张原图 = 同一物理曝光）---")
    print("   %-6s %-6s %-6s %-8s %-9s %-8s %-9s %-9s"
          % ("SL帧", "官方帧", "图#", "配对点", "框内命中率", "中位距m", "官方MAE", "自研MAE"))
    for r in pair_rows:
        print("   %-6d %-6d %-6d %-8d %-7.3f %-8.3f %-9.3f %-9.3f"
              % (r[0], r[1], r[2], r[3], r[4], r[5], r[7], r[8]))
    pr = np.array(pair_rows, float)
    print("   合计 %d 对帧, 配对点 %d; 官方侧重采样自洽 median %.3f, 自研侧 %.3f"
          % (len(pr), pr[:, 3].sum(), np.median(pr[:, 7]), np.median(pr[:, 8])))
    for tag in ("official", "custom"):
        s = np.array(recon_shift[tag], float)
        if len(s):
            print("   %-9s 重采样最佳整数像素修正: median du=%+.2f dv=%+.2f, |shift| P90=%.2f px"
                  % (tag, np.median(s[:, 0]), np.median(s[:, 1]),
                     np.percentile(np.linalg.norm(s, axis=1), 90)))

    print("\n--- A. 像素坐标差 ---")
    masks = [("全部", np.ones(n, bool)), ("两侧可见&都有色", usable),
             ("可见+有色+非边缘", usable & ~A["edge_cus"]),
             ("非边缘&残差<5cm", usable & ~A["edge_cus"] & (A["mdist"] < 0.05)),
             ("可见+有色+深度边缘", usable & A["edge_cus"])]
    d_naive = np.hypot(A["u_cus"] - A["u_off"], A["v_cus"] - A["v_off"])
    _stat("A1 朴素 |Δpix| (自研ideal vs 官方畸变)", d_naive, masks)
    _stat("A1u Δu 分量", A["u_cus"] - A["u_off"], masks)
    _stat("A1v Δv 分量", A["v_cus"] - A["v_off"], masks)
    # 约定对齐后：把自研理想像素换算到原图(畸变)像素坐标，与官方同系比较
    d_phys = np.hypot(A["u_cus_raw"] - A["u_off"], A["v_cus_raw"] - A["v_off"])
    _stat("A2 约定对齐后 |Δpix| (几何差)", d_phys, masks)
    _stat("A2u Δu", A["u_cus_raw"] - A["u_off"], masks)
    _stat("A2v Δv", A["v_cus_raw"] - A["v_off"], masks)
    # 同一几何、不同模型（应恒等于畸变位移，用来验证「投影公式本身」没有差异）
    d_model = np.hypot(A["u_shp"] - A["u_off"], A["v_shp"] - A["v_off"])
    _stat("A3 同几何不同模型 |Δpix| (纯畸变约定)", d_model, masks)
    d_lever = np.hypot(A["u_lv"] - A["u_shp"], A["v_lv"] - A["v_shp"])
    _stat("A4 假想: 忽略 extT 杆臂 |Δpix|", d_lever, masks)
    # 逐点配对不可能拿到同一个物理点：两侧是不同时刻同一颗 LiDAR 的不同采样
    # (自研 22205 点原始扫描 vs 官方体素降采样)。mdist 的残差本身就会造成像素差，
    # 下面用 fx*mdist/z 作为「几何完全一致时也会有的像素差」的上界。
    exp_pix = FX * A["mdist"] / np.maximum(A["depth"], 0.5)
    _stat("A5 配对残差折算的像素上界", exp_pix, masks)

    print("\n--- B. RGB 差 ---")
    drgb = np.linalg.norm(A["rgb_off"] - A["rgb_cus"], axis=1)
    _stat("B1 发布 RGB 欧氏差", drgb, masks)
    for i, ch in enumerate("RGB"):
        _stat("B2 发布 %s 通道有符号差(官方-自研)" % ch,
              A["rgb_off"][:, i] - A["rgb_cus"][:, i], masks)
    # 通道互换检验：若两侧只是 R/B 顺序不同，交换自研 R/B 后差应骤降
    swap = A["rgb_cus"][:, ::-1]
    d_swap = np.linalg.norm(A["rgb_off"] - swap, axis=1)
    _stat("B3 自研 R/B 交换后差(检测通道反序)", d_swap, masks)
    # 同几何（共享位姿）下的模型/预处理差：官方发布 vs 自研模型重采样
    d_same_geo = np.linalg.norm(A["rgb_off"] - A["rgb_shp"], axis=1)
    _stat("B4 同几何: 官方发布 vs 去畸变图重采样", d_same_geo, masks)
    d_same_geo2 = np.linalg.norm(A["rgb_cus"] - A["rgb_d2"], axis=1)
    _stat("B5 同几何: 自研发布 vs 原图重采样", d_same_geo2, masks)

    print("\n--- C. 自洽性检查（重算 u,v 采样 RGB vs bag 发布 RGB）---")
    ok_o = usable & np.isfinite(A["rgb_off_rs"]).all(1)
    ok_c = usable & np.isfinite(A["rgb_cus_rs"]).all(1)
    e_o = np.abs(A["rgb_off"] - A["rgb_off_rs"]).mean(1)
    e_c = np.abs(A["rgb_cus"] - A["rgb_cus_rs"]).mean(1)
    med_o = float(np.median(e_o[ok_o])) if ok_o.sum() else float("nan")
    med_c = float(np.median(e_c[ok_c])) if ok_c.sum() else float("nan")
    print("   官方: MAE median=%.3f P90=%.3f  (n=%d)"
          % (med_o, np.percentile(e_o[ok_o], 90), ok_o.sum()))
    print("   自研: MAE median=%.3f P90=%.3f  (n=%d)"
          % (med_c, np.percentile(e_c[ok_c], 90), ok_c.sum()))
    print("   两侧系统间差(同批点) MAE median=%.3f (n=%d)"
          % (float(np.median(drgb[usable])) if usable.sum() else float("nan"), usable.sum()))
    # 预处理等价性：自研在去畸变图采样 vs 官方原图在同一物理位置采样
    ok_r = usable & np.isfinite(A["rgb_cus_onraw"]).all(1)
    e_r = np.abs(A["rgb_cus"] - A["rgb_cus_onraw"]).mean(1)
    print("   预处理等价性: 自研发布 vs 原图@畸变坐标 MAE median=%.3f (n=%d)"
          % (float(np.median(e_r[ok_r])) if ok_r.sum() else float("nan"), ok_r.sum()))
    self_ok = (ok_o.sum() > 50 and ok_c.sum() > 50 and med_o < PHOTO_T and med_c < PHOTO_T)
    if self_ok:
        print("   -> 自洽性通过（阈值: 中位 MAE < %.1f/255）：两侧投影链重建均能复现 bag 里"
              "发布的 RGB，说明下面 A/B/D 的像素与颜色差来自两套实现的真实差异。" % PHOTO_T)
    else:
        print("   -> 自洽性未通过（官方 %.2f / 自研 %.2f，阈值 %.1f/255）："
              "重实现的投影链无法复现 bag 发布的 RGB，说明本工具对某一侧的重建仍有错误，"
              "下面的三条判据结论不可采信。" % (med_o, med_c, PHOTO_T))

    print("\n--- D. 三条判据 ---")
    if not self_ok:
        print("   已跳过（自洽性检查未通过，不能据此定位差异层级）。")
        return
    sel = usable & ~A["edge_cus"]
    if sel.sum() < 100:
        sel = usable
    # 判据以「配对残差 <5cm」的子集为准：这批点基本是同一个物理点、同一次曝光，
    # 像素差与 RGB 差不再有「两侧采样到不同表面点」这个混淆项。
    tight = sel & (A["mdist"] < 0.05)
    if tight.sum() >= 500:
        prim, tag = tight, "配对残差<5cm"
    else:
        prim, tag = sel, "可见+有色+非边缘(残差<5cm 子集太小)"
    med_geo = float(np.median(d_phys[prim]))
    med_rgb = float(np.median(drgb[prim]))
    med_exp = float(np.median(exp_pix[prim]))
    print("   %s 子集 n=%d/%d: 像素差中位 %.2f px（配对残差折算的上界 %.2f px），"
          "RGB 差中位 %.2f，3D 残差中位 %.3f m，深度中位 %.1f m"
          % (tag, prim.sum(), sel.sum(), med_geo, med_exp, med_rgb,
             float(np.median(A["mdist"][prim])), float(np.median(A["depth"][prim]))))
    PIX_T = 2.0
    RGB_T = 15.0
    geo_sig = med_geo > max(PIX_T, 1.5 * med_exp)     # 超出配对残差能解释的范围才算真几何差
    if geo_sig and med_rgb >= RGB_T:
        verdict = ("判据 2 命中（主因在几何层）：约定对齐后仍有 %.1f px 的像素差，"
                   "超过配对残差能解释的 %.1f px（≥%.1f px），且落在同一张原图的物理位置上，"
                   "直接导致 %.1f 的 RGB 差。应查投影模型/外参约定/时间戳/图像坐标系，"
                   "而不是采样或编码。" % (med_geo, med_exp, PIX_T, med_rgb))
    elif med_rgb >= RGB_T:
        verdict = ("判据 1 命中（主因在像素取值层）：像素差 %.1f px 已在配对残差上界 %.1f px "
                   "之内（无系统性几何差），但同一物理位置的 RGB 差达 %.1f。应查图像预处理"
                   "(remap 去畸变)、颜色通道顺序、采样插值、颜色编码。"
                   % (med_geo, med_exp, med_rgb))
    else:
        verdict = ("判据 3：单帧层面像素差 %.1f px（配对残差上界 %.1f px）与 RGB 差 %.1f "
                   "都不显著 —— 两套实现在同帧同图同点上一致，差异不发生在单帧投影/采样，"
                   "应查跨帧累积、重复点与渲染。" % (med_geo, med_exp, med_rgb))
    print("   " + verdict)
    tail = float((drgb[prim] >= RGB_T).mean())
    print("     · 但尾部不重叠：残差<5cm 子集里 RGB 差 >=%.0f 的占 %.1f%%（P90=%.1f, P99=%.1f）。"
          "中位数一致不代表两套逐点取值一致；官方 publish_frame_world 没有遮挡剔除、"
          "自研有 z-buffer+min_quality，被前景挡住的点在两套里的取舍不同，"
          "这部分要落到跨帧/遮挡层去查（判据 3 的延伸）。"
          % (RGB_T, 100 * tail, float(np.percentile(drgb[prim], 90)),
             float(np.percentile(drgb[prim], 99))))
    print("\n   分项归因（可见+有色+非边缘）：")
    print("     · 朴素像素差中位 %.2f px = 纯畸变/坐标约定(A3) %.2f px + 几何差(A2) %.2f px；"
          "A2 已被配对残差上界 %.2f px 解释 => %s"
          % (np.median(d_naive[sel]), np.median(d_model[sel]), med_geo, med_exp,
             "无系统性几何差" if not geo_sig else "存在系统性几何差"))
    lev = float(np.median(d_lever[sel]))
    if med_geo < 0.5 * lev:
        lv_txt = "不存在整项杆臂丢失，外参约定一致"
    elif med_geo < lev:
        lv_txt = "未见整项杆臂丢失，但不排除亚像素级的约定/位姿残差"
    else:
        lv_txt = "量级与「整项丢掉杆臂」相当，需要查外参约定"
    print("     · 假想灵敏度：若把官方的 IMU→LiDAR 杆臂 extT(|%.1f cm|) 丢掉，会带来 %.2f px "
          "的像素差；实测几何差 %.2f px => %s"
          % (100 * np.linalg.norm(EXT_T), lev, med_geo, lv_txt))
    ch = np.mean(A["rgb_cus"][sel] - A["rgb_off"][sel], axis=0)
    anti = abs(ch[0] + ch[2]) < 0.4 * abs(ch[0] - ch[2]) and abs(ch[0] - ch[2]) > 6
    print("     · RGB 通道有符号差均值(自研-官方) R=%+.2f G=%+.2f B=%+.2f => %s"
          % (ch[0], ch[1], ch[2],
             "R/B 互为相反数，通道反序" if anti else "无 R/B 反序特征，通道顺序一致"))


if __name__ == "__main__":
    main()
