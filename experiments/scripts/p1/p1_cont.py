#!/usr/bin/env python3
"""轨迹连续性/发散度量（按 header stamp 排序后的真实轨迹顺序）。

用法: python3 /tmp/p1_cont.py <bagdir> <label>
打印一行：
  CONT <label> n=.. order_inv=.. inv_max=..s dpjump=..m@..s dpjump_p99=.. n_jump5cm=..
        n_jump20cm=.. dyawjump=..deg@..s nan=.. tspan=..s
"""
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
    ts, pos, quat, rcv = [], [], [], []
    for (stm, d) in con.execute(
            'select timestamp, data from messages where topic_id=? order by timestamp', (tid,)):
        c = CDR(d)
        t = c.i32() + c.u32() * 1e-9
        c.string()
        c.string()
        pos.append(c.f64(3))
        quat.append(c.f64(4))
        ts.append(t)
        rcv.append(stm * 1e-9)
    con.close()
    return np.array(ts), np.array(pos), np.array(quat), np.array(rcv)


def yaw(q):
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


bag = sys.argv[1]
label = sys.argv[2] if len(sys.argv) > 2 else bag
t, p, q, rcv = read(bag)
n = len(t)
if n < 10:
    print('CONT %s 数据太少(%d)' % (label, n))
    sys.exit(0)

# 记录顺序（=发布顺序）里 header stamp 回退的次数与最大幅度
inv = np.diff(t)
n_inv = int((inv < -1e-6).sum())
inv_max = float(-inv.min()) if n_inv else 0.0

# 真轨迹顺序：按 header stamp 排序
o = np.argsort(t, kind='stable')
ts, ps = t[o], p[o]
qs = q[o]
d2 = np.diff(ts)
dp = np.linalg.norm(np.diff(ps, axis=0), axis=1)
dy = np.abs(wrap(np.diff(yaw(qs))))
dmed = float(np.median(d2)) if d2.size else 0.1
ok = (d2 > 1e-6) & (d2 < 1.5 * dmed)      # 正常节拍
dpn, dyn, dtn = dp[ok], dy[ok], d2[ok]
k = int(np.argmax(dpn)) if dpn.size else 0
ky = int(np.argmax(dyn)) if dyn.size else 0
bad = ~np.isfinite(ps).all(axis=1)
nan = int((~np.isfinite(dp)).sum() + (~np.isfinite(dyn)).sum()) + int(bad.sum())
print('CONT %s n=%d order_inv=%d inv_max=%.3fs beat=%.3fs dpjump=%.3fm@%.1fs(dp=%.3fs) '
      'dpjump_p99=%.3f n_jump5cm=%d n_jump20cm=%d dyawjump=%.2fdeg@%.1fs nan=%d tspan=%.1fs'
      % (label, n, n_inv, inv_max, dmed, dpn[k], ts[k], dtn[k],
         float(np.percentile(dpn, 99)) if dpn.size else 0.0,
         int((dpn > 0.05).sum()), int((dpn > 0.20).sum()),
         dyn[ky], ts[ky], nan, float(ts[-1] - ts[0])))
