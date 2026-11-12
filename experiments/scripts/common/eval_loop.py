#!/usr/bin/env python3
"""闭环评测：与 fastlivo2_bag_yaw_drift_report.html 同一口径。
   回环 = t>20 s 中距起点最近的帧，与第 0 帧比位置与 yaw；终点同理。
用法: python3 eval_loop.py <bag_dir> [label]"""
import sqlite3
import struct
import sys
from pathlib import Path

import numpy as np


class CDR:
    def __init__(self, b):
        self.b, self.o = b, 4

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

    def f64(self, n=1):
        self._a(8)
        v = struct.unpack_from('<%dd' % n, self.b, self.o)
        self.o += 8 * n
        return np.array(v) if n > 1 else v[0]

    def string(self):
        n = self.u32()
        s = self.b[self.o:self.o + n]
        self.o += n
        return s.rstrip(b'\x00').decode('utf-8', 'replace')


def read(bag):
    db = sorted(Path(bag).glob('*.db3'))[0]
    con = sqlite3.connect(str(db))
    tid = con.execute("select id from topics where name='/lio/odom'").fetchone()[0]
    ts, pos, quat = [], [], []
    for (d,) in con.execute('select data from messages where topic_id=? order by timestamp', (tid,)):
        c = CDR(d)
        t = c.i32() + c.u32() * 1e-9
        c.string()
        c.string()
        pos.append(c.f64(3))
        quat.append(c.f64(4))
        ts.append(t)
    con.close()
    return np.array(ts), np.array(pos), np.array(quat)


def yaw(q):
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


bag = sys.argv[1]
label = sys.argv[2] if len(sys.argv) > 2 else bag
t, p, q = read(bag)
if len(t) < 10:
    print('%s: 数据太少 (%d 帧)' % (label, len(t)))
    sys.exit(1)

t = t - t[0]
y = yaw(q)
d = np.linalg.norm(p - p[0], axis=1)
path = np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))
mask = t > 20.0
i = np.argmin(np.where(mask, d, 1e9))
print('== %s' % label)
print('  帧数 %d  时长 %.1f s  路径长 %.2f m' % (len(t), t[-1], path))
print('  回环 t=%.1f s  位置偏差 %.2f m  yaw 偏差 %+.1f°  (距起点最小 %.2f m)'
      % (t[i], d[i], wrap(y[i] - y[0]), d[i]))
print('  终点 位置偏差 %.2f m  yaw 偏差 %+.1f°' % (d[-1], wrap(y[-1] - y[0])))
print('  yaw 范围 %.1f°  端点距离 %.2f m' % (np.abs(wrap(y - y[0])).max(), d[-1]))
# 逐 10 s 打印一次 yaw 与距起点距离，便于贴到报告里
print('  t(s)   x      y      z     dist  yaw(deg)')
for k in range(0, len(t), max(1, len(t) // 16)):
    print('  %5.1f  %6.2f %6.2f %6.2f  %5.2f  %+7.1f'
          % (t[k], p[k, 0], p[k, 1], p[k, 2], d[k], wrap(y[k] - y[0])))
