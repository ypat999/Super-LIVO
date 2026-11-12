#!/usr/bin/env python3
"""逐帧比较两组 /lio/odom 的位置差（按索引对齐，非时间对齐）。"""
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
    ts, pos = [], []
    for (d,) in con.execute('select data from messages where topic_id=? order by timestamp', (tid,)):
        c = CDR(d)
        t = c.i32() + c.u32() * 1e-9
        c.string()
        c.string()
        pos.append(c.f64(3))
        c.f64(4)
        ts.append(t)
    con.close()
    return np.array(ts), np.array(pos)


base, other = sys.argv[1], sys.argv[2]
t1, p1 = read(base)
t2, p2 = read(other)
n = min(len(p1), len(p2))
d = np.linalg.norm(p1[:n] - p2[:n], axis=1)
print('idx   t_off    |dp|(m)')
for k in range(0, n, max(1, n // 12)):
    print('%5d %7.1f  %8.4f' % (k, t1[k] - t1[0], d[k]))
print('逐帧 |dp|: 中位 %.4f m  90%% %.4f m  最大 %.4f m (idx=%d)'
      % (np.median(d), np.percentile(d, 90), d.max(), int(np.argmax(d))))
