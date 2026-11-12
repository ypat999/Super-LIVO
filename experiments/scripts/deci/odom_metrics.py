#!/usr/bin/env python3
"""从 /lio/odom bag 提取轨迹级指标（room 回环：终点相对起点的位移与净 yaw）。"""
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
    print('  ODOM %s 数据太少(%d)' % (label, len(t)))
    sys.exit(0)

dt = np.diff(t)
dp = np.linalg.norm(np.diff(p, axis=0), axis=1)
dyw = wrap(np.diff(yaw(q)))
dmed = float(np.median(dt))
ok = dt < 1.5 * dmed
yy = yaw(q)
# 累积（解缠）yaw：闭环回原地应当回到 0；偏差即 yaw 漂移
cum_yaw = float(np.cumsum(dyw)[ok][-1]) if ok.any() else float(np.cumsum(dyw)[-1])
v = dp / np.maximum(dt, 1e-6)
print('  ODOM %s frames=%d dur=%.1fs path=%.1fm end_dist=%.2fm '
      'yaw_drift=%.2fdeg dp_med=%.3f dp_max=%.2f@%.0fs v_max=%.1f dyaw_max=%.1fdeg '
      'n_dt_skip=%d' % (
          label, len(t), t[-1] - t[0], float(dp.sum()),
          float(np.linalg.norm(p[-1] - p[0])), cum_yaw,
          float(np.median(dp[ok])), float(dp[ok].max()),
          float(t[1:][ok][np.argmax(dp[ok])] - t[0]),
          float(v[ok].max()), float(np.abs(dyw[ok]).max()), int((~ok).sum())))
