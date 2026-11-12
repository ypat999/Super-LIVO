#!/usr/bin/env python3
"""对比 b2 / off 两次全量跑的逐帧运动量（按 header stamp 对齐），
   定位 b2 路径长 755 m vs off 231 m 的差异来源。"""
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


def load(d):
    t, p, q = read(d)
    o = np.argsort(t)
    return {'t': t[o] - t[o][0], 'pos': p[o], 'yaw': yaw(q[o])}


def stats(tag, a):
    p, y, t = a['pos'], a['yaw'], a['t']
    dp = np.linalg.norm(np.diff(p, axis=0), axis=1)
    dy = np.diff(y)
    dy = (dy + 180) % 360 - 180
    print(f'== {tag}: frames={len(p)} dur={t[-1]:.1f}s path={dp.sum():.2f}m')
    print(f'   |dp|/frm mean={dp.mean()*1000:.1f}mm p50={np.median(dp)*1000:.1f}mm '
          f'p90={np.percentile(dp,90)*1000:.1f}mm max={dp.max()*1000:.1f}mm')
    print(f'   |dyaw|/frm mean={np.abs(dy).mean():.4f}deg p90={np.percentile(np.abs(dy),90):.4f}deg '
          f'max={np.abs(dy).max():.3f}deg')


A = load(sys.argv[1])
B = load(sys.argv[2])
stats('A ' + sys.argv[3], A)
stats('B ' + sys.argv[4], B)

ta, tb = A['t'], B['t']
idx = np.clip(np.searchsorted(tb, ta), 0, len(tb) - 1)
ok = np.abs(tb[idx] - ta) < 0.005
ii = np.where(ok)[0]
dp = np.linalg.norm(A['pos'][ii] - B['pos'][idx[ii]], axis=1)
dy = (A['yaw'][ii] - B['yaw'][idx[ii]] + 180) % 360 - 180
print(f'== stamp对齐 {len(ii)}/{len(ta)}: |dpos| mean={dp.mean()*1000:.1f}mm '
      f'p90={np.percentile(dp,90)*1000:.1f}mm max={dp.max()*1000:.1f}mm | '
      f'|dyaw| mean={np.abs(dy).mean()*1000:.2f}mrad max={np.abs(dy).max():.3f}deg')
for lo, hi in ((0, 40), (40, 80), (80, 120), (120, 160)):
    sel = (ta[ii] >= lo) & (ta[ii] < hi)
    if sel.sum():
        print(f'   [{lo:3d},{hi:3d})s dpos mean={dp[sel].mean()*1000:7.1f}mm max={dp[sel].max()*1000:8.1f}mm'
              f'  dyaw mean={np.abs(dy[sel]).mean()*1000:7.2f}mrad max={np.abs(dy[sel]).max():6.3f}deg'
              f'  (n={sel.sum()})')
