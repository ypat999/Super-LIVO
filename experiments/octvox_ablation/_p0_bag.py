#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Phase 0 共用的 bag 读取层（纯 sqlite3 + CDR 手解，不依赖 ROS 消息类型）。

为什么不用 rosbag2_py：`livox_ros_driver2/msg/CustomMsg` 的 python 绑定在本机
未安装（实测 ModuleNotFoundError），而 rosbag2_py 反序列化需要类型支持。
sqlite3 + CDR 直读在 tools/rtk_verify_custommsg.py 里已验证零剩余字节，
故本模块沿用同一套解析。

时钟域提醒（重要，P0-A 的核心）：
  base.txt 等 TUM 轨迹的时间戳（946685438 量级 = 2000-01-01 起算）
  与 bag 元数据的 starting_time（1664265583 量级 = 2022-09-27）**不在同一域**。
  所以任何"按绝对时刻匹配轨迹与图像"的写法都是错的，必须用**相对时间**
  （各自减去首帧）或显式求常值偏置。本模块提供 rel() 做这件事。
"""
import sqlite3
import struct

import numpy as np

BAG = '/home/ywj/slam_data/Retail_Street/Retail_Street.db3'
CAM = '/home/ywj/slam_data/Retail_Street/camera_pinhole.yaml'


class CDR:
    """ROS 2 CDR（little-endian，带 4 字节封装头）游标。"""

    def __init__(self, buf):
        self.b = bytes(buf)                     # sqlite3 默认把 BLOB 包成 memoryview，必须转 bytes
        assert self.b[0:2] == b'\x00\x01', 'expect CDR LE, got %r' % self.b[0:4]
        self.o = 4

    def _al(self, n):
        r = (self.o - 4) % n
        if r:
            self.o += n - r

    def i32(self):
        self._al(4)
        v = struct.unpack_from('<i', self.b, self.o)[0]
        self.o += 4
        return v

    def u32(self):
        self._al(4)
        v = struct.unpack_from('<I', self.b, self.o)[0]
        self.o += 4
        return v

    def u64(self):
        self._al(8)
        v = struct.unpack_from('<Q', self.b, self.o)[0]
        self.o += 8
        return v

    def f32(self):
        self._al(4)
        v = struct.unpack_from('<f', self.b, self.o)[0]
        self.o += 4
        return v

    def u8(self):
        v = self.b[self.o]
        self.o += 1
        return v

    def string(self):
        n = self.u32()
        s = self.b[self.o:self.o + n]
        self.o += n
        return s.rstrip(b'\x00').decode('utf-8', 'replace')


def _hdr(c):
    """std_msgs/Header -> float 秒。"""
    sec = c.i32()
    nsec = c.u32()
    frame_id = c.string()
    return sec + 1e-9 * nsec, frame_id


def topics(db3=BAG):
    con = sqlite3.connect(db3)
    rows = con.execute('select id,name,type from topics').fetchall()
    con.close()
    return {n: (i, t) for i, n, t in rows}


def _iter(db3, name):
    con = sqlite3.connect(db3)
    tid = con.execute('select id from topics where name=?', (name,)).fetchone()[0]
    cur = con.execute('select data,timestamp from messages where topic_id=? '
                      'order by timestamp', (tid,))
    for data, ts in cur:
        yield bytes(data), ts
    con.close()


def read_imu(db3=BAG):
    """sensor_msgs/Imu -> (t[N], gyro[N,3], acc[N,3])，t = header.stamp（秒）。

    Header 之后是 37 个 float64（实测报文长 324 B）：orientation(4) + cov(9)
    + angular_velocity(3) + cov(9) + linear_acceleration(3) + cov(9)。
    注意 Avia 的加速度单位是 g（|a|≈1.0），角速度 rad/s。
    """
    t, g, a = [], [], []
    for data, _ in _iter(db3, '/livox/imu'):
        c = CDR(data)
        ts, _ = _hdr(c)
        c._al(8)
        v = np.frombuffer(c.b, dtype='<f8', count=37, offset=c.o)
        t.append(ts)
        g.append(v[13:16])
        a.append(v[25:28])
    return np.array(t), np.ascontiguousarray(np.array(g)), np.ascontiguousarray(np.array(a))


PT_STRIDE = 20   # CustomPoint 的 CDR 步长：offset_time(u32)+xyz(3*f32)+refl/tag/line(3*u8)+1 填充
                 # 注意不是 19 —— 元素按 4 字节对齐，实测 24000 点/包正好占满 480 KB。


def read_lidar_stamps(db3=BAG, stride=1):
    """CustomMsg -> (t0, t_end, point_num)。

    t_end = header.stamp + max(offset_time)*1e-9（= Super-LIO 的 lidar.end_time，
    即 odom 时间戳语义）。offset_time 是 uint32 纳秒、相对 header.stamp。
    stride>1 只抽样点位以加速（offset_time 在帧内单调递增，抽样对 max 影响 <1 行周期）。
    """
    t0, te, pn = [], [], []
    for data, _ in _iter(db3, '/livox/lidar'):
        c = CDR(data)
        ts, _ = _hdr(c)
        c.u64()                                   # timebase
        point_num = c.u32()
        c.u8()                                    # lidar_id
        c.o += 3                                  # rsv[3]
        n = c.u32()
        base = c.o
        # 每点在 CDR 里占 20 B = 5 个 u32；offset_time 是第 0 个。
        # 直接 frombuffer(count=n) 会按 4 B 步进读到 x/y/z 的位模式（曾造成
        # scan_len≈3.2 s 的假象），必须按 5*u32 步进取。
        cnt = min(5 * n, (len(c.b) - base) // 4)   # 报文末尾缺 1 B 填充，按实际可读长度取
        offs = np.frombuffer(c.b, dtype='<u4', count=cnt, offset=base)[::5][::stride]
        t0.append(ts)
        te.append(ts + 1e-9 * int(offs.max()) if len(offs) else ts)
        pn.append(point_num)
    return np.array(t0), np.array(te), np.array(pn)


def read_image_stamps(db3=BAG):
    """sensor_msgs/Image -> header.stamp 序列（只解 header，便宜）。"""
    out = []
    for data, _ in _iter(db3, '/left_camera/image'):
        c = CDR(data)
        ts, _ = _hdr(c)
        out.append(ts)
    return np.array(out)


def _img_meta(data):
    c = CDR(data)
    ts, frame = _hdr(c)
    h = c.u32()
    w = c.u32()
    enc = c.string()
    c.u8()                                        # is_bigendian
    step = c.u32()
    n = c.u32()
    off = c.o
    return ts, frame, h, w, enc, step, n, off


def decode_image(data):
    ts, _, h, w, enc, step, nbytes, off = _img_meta(data)
    ch = 3 if ('rgb8' in enc or 'bgr8' in enc) else 1
    buf = np.frombuffer(data, dtype=np.uint8, count=step * h, offset=off)
    img = buf.reshape(h, step)[:, :w * ch]
    img = img.reshape(h, w, ch) if ch > 1 else img.reshape(h, w)
    return ts, np.ascontiguousarray(img), enc


def read_image_at(idx, db3=BAG):
    """按序取第 idx 张图，返回 (t, uint8 array[H,W,C or 1], encoding)。"""
    for k, (data, _) in enumerate(_iter(db3, '/left_camera/image')):
        if k == idx:
            return decode_image(data)
    raise IndexError(idx)


def read_images(idxs, db3=BAG):
    """单趟扫描取多张图：{idx: (t, img, enc)}。idxs 需为集合/列表。"""
    want = set(int(i) for i in idxs)
    out = {}
    if not want:
        return out
    for k, (data, _) in enumerate(_iter(db3, '/left_camera/image')):
        if k in want:
            out[k] = decode_image(data)
            want.discard(k)
            if not want:
                break
    missing = set(out) ^ set(int(i) for i in idxs)
    if missing:
        raise IndexError('bag 中不存在的图像下标 %s' % sorted(missing))
    return out
