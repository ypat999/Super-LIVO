#!/usr/bin/env python3
"""比较两个 /lio/odom 记录：同一时刻位姿差多少（用于判断某参数是否真的改变了结果）。"""
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
    for (d,) in con.execute("select data from messages where topic_id=? order by timestamp", (tid,)):
        c = CDR(d)
        t = c.i32() + c.u32() * 1e-9
        ts.append(t)
        c.string()
        c.string()
        pos.append(c.f64(3))
        quat.append(c.f64(4))
    con.close()
    return np.array(ts), np.array(pos), np.array(quat)


a = read(sys.argv[1])
b = read(sys.argv[2])
print('A: %d poses %.1f s   B: %d poses %.1f s' % (len(a[0]), a[0][-1] - a[0][0],
                                                   len(b[0]), b[0][-1] - b[0][0]))
n = min(len(a[0]), len(b[0]))
# 按时间最近邻配对（两轮 bag 播放在同一时间轴）
idx = np.searchsorted(b[0], a[0])
idx = idx.clip(0, len(b[0]) - 1)
dp = np.linalg.norm(a[1] - b[1][idx], axis=1)
print('配对 %d 帧 |Δp|:  max %.6f m   median %.6f m   p95 %.6f m'
      % (n, dp.max(), np.median(dp), np.percentile(dp, 95)))
print('最差 5 帧（用于区分"真发散"与"时间配对错位"）:')
for i in np.argsort(-dp)[:5]:
    print('  t=%.3f  |Δp|=%.4f m   A=%s  B=%s'
          % (a[0][i], dp[i], np.round(a[1][i], 3), np.round(b[1][idx[i]], 3)))
print('  A 末位姿 %s' % (np.round(a[1][-1], 4),))
print('  B 末位姿 %s' % (np.round(b[1][-1], 4),))
